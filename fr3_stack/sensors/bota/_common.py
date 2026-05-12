"""Shared helpers for FT calibration / compensation entry points."""
from __future__ import annotations

import os
import time
from pathlib import Path

import numpy as np
import yaml

from fr3_stack import Robot

GRAVITY = 9.80665
NUM_SAMPLES = 50            # samples averaged per pose
SAMPLE_PERIOD = 0.02        # 50 Hz polling
DEFAULT_HOST = "192.168.1.8"


def default_config_dir() -> Path:
    """In-tree calib dir: keeps daemon, Python tools, and docker volume mount
    anchored at one path. Override via ``$FR3_FT_CALIB_DIR``."""
    override = os.environ.get("FR3_FT_CALIB_DIR")
    if override:
        return Path(override)
    return Path(__file__).resolve().parent / "config"


def default_calib_path() -> Path:
    return default_config_dir() / "ft_calibration.yaml"


def default_waypoints_path() -> Path:
    return default_config_dir() / "ft_calibration_waypoints.yaml"


def quat_xyzw_to_R(q: np.ndarray) -> np.ndarray:
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w)],
        [2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y)],
    ])


def rpy_to_R(rpy: np.ndarray) -> np.ndarray:
    """ZYX intrinsic — matches the C++ loader and scipy."""
    r, p, y = float(rpy[0]), float(rpy[1]), float(rpy[2])
    cr, sr = np.cos(r), np.sin(r)
    cp, sp = np.cos(p), np.sin(p)
    cy, sy = np.cos(y), np.sin(y)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def load_mount_rpy(calib_path: Path) -> np.ndarray:
    """Read ``rpy_ee_sensor`` from an existing calib YAML; ``[0,0,0]`` if absent.

    Why: on rigs with a Desk hand configured, libfranka's EE frame is rotated
    vs the bota sensor frame and the calibration must compensate. See
    docs/postmortems/ft_calibration_2026-05-10.md.
    """
    if not calib_path.exists():
        return np.zeros(3, dtype=float)
    try:
        with open(calib_path) as fp:
            doc = yaml.safe_load(fp) or {}
    except (OSError, yaml.YAMLError):
        return np.zeros(3, dtype=float)
    raw = doc.get("rpy_ee_sensor")
    if raw is None or len(raw) != 3:
        return np.zeros(3, dtype=float)
    return np.asarray(raw, dtype=float)


def collect_pose(
    robot: Robot,
    R_ee_sensor: np.ndarray | None = None,
) -> tuple[np.ndarray, np.ndarray, np.ndarray] | None:
    """Average NUM_SAMPLES wrench/quat readings; returns
    ``(R_O_sensor, f_sensor, t_sensor)`` or None if too few valid samples.

    Reads ``wrench_ft_raw`` — never ``wrench_ft`` — to avoid fitting on
    data already compensated against a previous calib (would converge to
    mass≈0). See docs/postmortems/ft_calibration_2026-05-10.md.
    """
    if R_ee_sensor is None:
        R_ee_sensor = np.eye(3)
    fs: list[np.ndarray] = []
    ts: list[np.ndarray] = []
    qs: list[np.ndarray] = []
    n_skipped = 0
    for _ in range(NUM_SAMPLES):
        s = robot.state
        if not s.valid or not s.has_ft_sensor:
            n_skipped += 1
            time.sleep(SAMPLE_PERIOD)
            continue
        R_O_sensor = quat_xyzw_to_R(s.quat_xyzw) @ R_ee_sensor
        # Must use raw stream. Three cases:
        #   (a) wrench_ft_raw on the wire — normal.
        #   (b) old daemon, no calib loaded — wrench_ft IS the raw stream.
        #   (c) old daemon WITH calib loaded — refuse: would fit delta vs
        #       the loaded calib and produce mass≈0.
        if s.wrench_ft_raw is not None:
            w = s.wrench_ft_raw
        elif not s.ft_compensated:
            w = s.wrench_ft
        else:
            raise RuntimeError(
                "daemon publishes wrench_ft (compensated) but not "
                "wrench_ft_raw. Calibrating on compensated data fits the "
                "DELTA against the loaded calib (gives mass≈0 or negative). "
                "Workaround: move ft_calibration.yaml out of the daemon's "
                "calib dir, restart the daemon (it'll boot uncompensated, "
                "ft_compensated=False, wrench_ft becomes the raw stream), "
                "re-run this calibration, then move the new yaml back."
            )
        fs.append(R_O_sensor.T @ w[0:3])
        ts.append(R_O_sensor.T @ w[3:6])
        qs.append(s.quat_xyzw.copy())
        time.sleep(SAMPLE_PERIOD)

    if len(fs) < NUM_SAMPLES // 5:
        print(f"  not enough samples ({len(fs)}/{NUM_SAMPLES}, skipped {n_skipped})")
        return None

    q_avg = np.mean(qs, axis=0)
    R_O_sensor = quat_xyzw_to_R(q_avg / np.linalg.norm(q_avg)) @ R_ee_sensor
    f_avg = np.mean(fs, axis=0)
    t_avg = np.mean(ts, axis=0)
    f_std = np.std(np.asarray(fs), axis=0)
    t_std = np.std(np.asarray(ts), axis=0)
    g_in_sensor = R_O_sensor.T @ np.array([0.0, 0.0, 1.0])
    print(f"  Force  (sensor):  [{f_avg[0]:+.4f}, {f_avg[1]:+.4f}, {f_avg[2]:+.4f}] N   "
          f"|F|={np.linalg.norm(f_avg):.3f} N")
    print(f"  Force  std:       [{f_std[0]:.4f}, {f_std[1]:.4f}, {f_std[2]:.4f}] N "
          f"(>0.5N likely still moving)")
    print(f"  Torque (sensor):  [{t_avg[0]:+.6f}, {t_avg[1]:+.6f}, {t_avg[2]:+.6f}] Nm")
    print(f"  Torque std:       [{t_std[0]:.4f}, {t_std[1]:.4f}, {t_std[2]:.4f}] Nm")
    print(f"  g_sensor (R^T·z): [{g_in_sensor[0]:+.4f}, {g_in_sensor[1]:+.4f}, "
          f"{g_in_sensor[2]:+.4f}]  (unit gravity dir in sensor frame)")
    return R_O_sensor, f_avg, t_avg


def solve(poses: list[tuple[np.ndarray, np.ndarray, np.ndarray]]) -> dict | None:
    """Two-stage LSQ for {mass, CoM, force_bias, torque_bias}.

    Stage 1: ``f_meas = -m·R^T·e3 + f_bias`` → (m·g, f_bias).
    Stage 2: ``t_meas = CoM × g_sensor + t_bias`` with mass fixed.
    """
    n = len(poses)
    if n < 3:
        print(f"need >=3 poses, got {n}")
        return None
    e3 = np.array([0.0, 0.0, 1.0])

    # Diag: if R^T·e3 doesn't span the sphere, mass becomes unidentifiable
    # (LSQ dumps gravity into f_bias and returns mass≈0). Span < 0.5/axis is red.
    G = np.stack([R.T @ e3 for R, _, _ in poses])  # n x 3
    g_min, g_max = G.min(axis=0), G.max(axis=0)
    g_span = g_max - g_min
    print(f"\n  gravity-vector spread across {n} poses (sensor frame):")
    print(f"    x: [{g_min[0]:+.3f} .. {g_max[0]:+.3f}]   span {g_span[0]:.3f}")
    print(f"    y: [{g_min[1]:+.3f} .. {g_max[1]:+.3f}]   span {g_span[1]:.3f}")
    print(f"    z: [{g_min[2]:+.3f} .. {g_max[2]:+.3f}]   span {g_span[2]:.3f}")
    g_sv = np.linalg.svd(G - G.mean(axis=0), compute_uv=False)
    print(f"    centered SVs: [{g_sv[0]:.3f}, {g_sv[1]:.3f}, {g_sv[2]:.3f}]  "
          "(small smallest SV ⇒ poses lie on a plane/line ⇒ mass degenerate)")

    A_f = np.zeros((3 * n, 4))
    b_f = np.zeros(3 * n)
    for i, (R, f_meas, _) in enumerate(poses):
        g_in_sensor = R.T @ e3
        A_f[3 * i:3 * i + 3, 0]   = -g_in_sensor
        A_f[3 * i:3 * i + 3, 1:4] = np.eye(3)
        b_f[3 * i:3 * i + 3]      = f_meas

    # Diag: cond(A_f) > 1e3 ⇒ mass/bias indistinguishable; add wrist-tilted poses.
    sv = np.linalg.svd(A_f, compute_uv=False)
    cond = sv[0] / max(sv[-1], 1e-30)
    print(f"  A_f singular values: {[f'{s:.3f}' for s in sv]}")
    print(f"  A_f cond = {cond:.1f}   "
          f"({'OK' if cond < 100 else 'POOR' if cond < 1000 else 'DEGENERATE'})")

    x_f, *_ = np.linalg.lstsq(A_f, b_f, rcond=None)
    mg = float(x_f[0])
    mass = mg / GRAVITY
    f_bias = x_f[1:4]

    A_t = np.zeros((3 * n, 6))
    b_t = np.zeros(3 * n)
    for i, (R, _, t_meas) in enumerate(poses):
        g_sensor = -mg * (R.T @ e3)
        gx, gy, gz = g_sensor
        # com × g = skew(g)^T @ com → linear in com.
        A_t[3 * i:3 * i + 3, 0:3] = np.array([
            [  0,  gz, -gy],
            [-gz,   0,  gx],
            [ gy, -gx,   0],
        ])
        A_t[3 * i:3 * i + 3, 3:6] = np.eye(3)
        b_t[3 * i:3 * i + 3]      = t_meas
    x_t, *_ = np.linalg.lstsq(A_t, b_t, rcond=None)
    com    = x_t[0:3]
    t_bias = x_t[3:6]

    f_err: list[float] = []
    t_err: list[float] = []
    for R, f_meas, t_meas in poses:
        g_sensor = -mg * (R.T @ e3)
        f_err.append(float(np.linalg.norm(f_meas - (g_sensor + f_bias))))
        t_err.append(float(np.linalg.norm(t_meas - (np.cross(com, g_sensor) + t_bias))))

    warnings: list[str] = []
    if mass <= 0.0:
        warnings.append(
            f"mass={mass:+.4f} kg is non-positive — likely opposite force-sign "
            "convention or insufficient EE-orientation diversity (vary joints "
            "1/4/6/7 so R^T·e3 spans more of the unit sphere)"
        )
    if 0.0 < mass < 0.020:
        warnings.append(
            f"mass={mass*1000:.1f} g is implausibly small for a Bota + adapter — "
            "the LSQ has likely dumped gravity into f_bias. Check the gravity "
            "spread above (each axis needs span >~0.7 to identify mass) and "
            "the A_f cond (>1000 means mass is degenerate with bias)."
        )
    if float(np.linalg.norm(com)) > 0.5:
        warnings.append(
            f"|CoM|={float(np.linalg.norm(com)):.3f} m exceeds 0.5 m — "
            "implausible for a tool mounted on the FT flange"
        )
    # |f_bias| post-tare should be <2 N; >5 N suggests tare() failed (driver
    # returns false on EtherCAT timing hiccup) or LSQ absorbed gravity.
    fbn = float(np.linalg.norm(f_bias))
    if fbn > 5.0:
        warnings.append(
            f"|f_bias|={fbn:.2f} N is large (typical post-tare is <2 N). "
            "The bota auto-tare may have failed silently — check fr3d boot "
            "stdout for '[fr3] bota: tare() failed (continuing)'. If tare did "
            "run, f_bias being this large means the LSQ absorbed gravity into "
            "bias (insufficient orientation spread; see diagnostics above)."
        )

    result = {
        "mass":            float(mass),
        "center_of_mass":  [float(c) for c in com],
        "force_bias":      [float(b) for b in f_bias],
        "torque_bias":     [float(b) for b in t_bias],
        "num_poses":       n,
        "mean_force_residual_N":   float(np.mean(f_err)),
        "mean_torque_residual_Nm": float(np.mean(t_err)),
    }
    if warnings:
        result["warnings"] = warnings
    return result


def print_result(result: dict) -> None:
    com    = result["center_of_mass"]
    f_bias = result["force_bias"]
    t_bias = result["torque_bias"]
    bar = "=" * 56
    print("\n" + bar)
    print("RESULT")
    print(bar)
    print(f"  mass         {result['mass']:.4f} kg")
    print(f"  CoM          [{com[0]:+.6f}, {com[1]:+.6f}, {com[2]:+.6f}] m")
    print(f"  force_bias   [{f_bias[0]:+.4f}, {f_bias[1]:+.4f}, {f_bias[2]:+.4f}] N")
    print(f"  torque_bias  [{t_bias[0]:+.6f}, {t_bias[1]:+.6f}, {t_bias[2]:+.6f}] Nm")
    print(f"  residual:    force={result['mean_force_residual_N']:.4f} N   "
          f"torque={result['mean_torque_residual_Nm']:.6f} Nm")
    print(bar)
    if result.get("warnings"):
        print("WARNINGS — calibration is not physically plausible:")
        for w in result["warnings"]:
            print(f"  - {w}")
        print("Re-record with more diverse EE orientations and re-run.")
        print(bar)


def save_yaml(data: dict, path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w") as fp:
        yaml.dump(data, fp, default_flow_style=False, sort_keys=False)
