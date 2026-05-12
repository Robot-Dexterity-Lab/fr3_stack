"""Recording + analysis for Python-side experiments.

Samples at ~200 Hz (daemon state rate). CSV's first 13 columns match
src/bin/cartesian_test.cpp's output so the same analyses apply to both.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

import numpy as np


_HEADER = ("t,p_d_x,p_d_y,p_d_z,p_x,p_y,p_z,Fx,Fy,Fz,Tx,Ty,Tz,"
           "F_ft_x,F_ft_y,F_ft_z,T_ft_x,T_ft_y,T_ft_z")


@dataclass
class Recorder:
    """In-memory ring of (t, p_d, p, F, F_ft) samples. Built by ``Arm.record()``.

    ``mode`` is set by the recording helper so ``metrics()`` / ``plot()`` pick
    the right view.
    """
    mode:    str            = "hold"     # "hold" | "osc" | "step" | "disturb"
    axis:    int            = 2
    rate_hz: float          = 100.0
    _rows:   list           = field(default_factory=list)
    _has_ft: bool           = False

    # ---- producer side (called by Arm helpers) ---------------------------

    def push(self, t: float, p_d: np.ndarray, obs) -> None:
        row = [float(t)]
        row.extend(float(x) for x in p_d)
        row.extend(float(x) for x in obs.pose.pos)
        # Keep both libfranka-est and FT columns regardless for a fixed schema.
        if obs.has_ft:
            self._has_ft = True
            row.extend([float("nan")] * 6)
            row.extend(float(x) for x in obs.wrench)
        else:
            row.extend(float(x) for x in obs.wrench)
            row.extend([float("nan")] * 6)
        self._rows.append(row)

    # ---- consumer side ---------------------------------------------------

    def __len__(self) -> int:
        return len(self._rows)

    def to_array(self) -> np.ndarray:
        if not self._rows:
            raise RuntimeError("recorder is empty")
        return np.asarray(self._rows)

    def save(self, path: str | Path) -> Path:
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        np.savetxt(path, self.to_array(), delimiter=",",
                   header=_HEADER, comments="", fmt="%.6f")
        return path

    def metrics(self) -> dict:
        """Mode-aware summary. Keys match cart_overall."""
        a = self.to_array()
        t   = a[:, 0]
        p_d = a[:, 1:4]
        p   = a[:, 4:7]
        F   = a[:, 7:10]
        out: dict = {"mode": self.mode, "n_samples": len(t),
                     "duration_s": float(t[-1] - t[0])}

        if self.mode in ("hold", "disturb"):
            drift = np.linalg.norm(p - p[0], axis=1) * 1000
            Fmag  = np.linalg.norm(F, axis=1)
            out.update(
                drift_mean_mm = float(drift.mean()),
                drift_max_mm  = float(drift.max()),
                F_mean_N      = float(Fmag.mean()),
                F_max_N       = float(Fmag.max()),
            )

        if self.mode == "osc":
            i = self.axis
            cmd  = p_d[:, i] - p_d[:, i].mean()
            meas = p[:, i]   - p[:, i].mean()
            err  = p[:, i]   - p_d[:, i]
            ratio = (meas.max() - meas.min()) / (cmd.max() - cmd.min() + 1e-12)
            n  = len(t)
            dt = float(np.median(np.diff(t)))
            c  = np.correlate(meas - meas.mean(), cmd - cmd.mean(), mode="full")
            lag_ms = float((np.argmax(c) - (n - 1)) * dt * 1000)
            out.update(
                amp_ratio       = float(ratio),
                err_rms_mm      = float(np.sqrt((err**2).mean()) * 1000),
                err_peak_mm     = float(np.abs(err).max() * 1000),
                phase_lag_ms    = lag_ms,
                cross_other1_mm = float((p[:, (i+1)%3].max() - p[:, (i+1)%3].min()) * 1000),
                cross_other2_mm = float((p[:, (i+2)%3].max() - p[:, (i+2)%3].min()) * 1000),
            )

        if self.mode == "circle":
            # Plane axes inferred from which p_d axis is constant.
            p_d_var = p_d.std(axis=0)
            third = int(np.argmin(p_d_var))
            i, j = [k for k in (0, 1, 2) if k != third]
            err_xy = p[:, [i, j]] - p_d[:, [i, j]]
            r_err  = np.linalg.norm(err_xy, axis=1) * 1000
            cmd_x = p_d[:, i] - p_d[:, i].mean()
            cmd_y = p_d[:, j] - p_d[:, j].mean()
            r_cmd = float(np.sqrt(cmd_x**2 + cmd_y**2).max() * 1000)
            # Measured radius about p mean — operator may have drifted.
            mx = p[:, i] - p[:, i].mean()
            my = p[:, j] - p[:, j].mean()
            r_meas = float(np.sqrt(mx**2 + my**2).max() * 1000)
            out.update(
                radius_cmd_mm    = r_cmd,
                radius_meas_mm   = r_meas,
                radius_ratio     = r_meas / r_cmd if r_cmd else float("nan"),
                radial_err_rms_mm  = float(np.sqrt((r_err**2).mean())),
                radial_err_peak_mm = float(r_err.max()),
            )

        if self.mode == "step":
            i = self.axis
            dpd = np.abs(np.diff(p_d[:, i]))
            if dpd.max() < 1e-4:
                out["error"] = "no step detected in p_d"
                return out
            i_step = int(np.argmax(dpd))
            t_step = float(t[i_step])
            p0 = float(p[:i_step, i].mean()) if i_step > 50 else float(p[0, i])
            p_target = float(p_d[-1, i])
            delta = p_target - p0

            tt = t[i_step:] - t_step
            pp = p[i_step:, i]

            crossed_10 = np.where((pp - p0) / delta >= 0.1)[0]
            crossed_90 = np.where((pp - p0) / delta >= 0.9)[0]
            t_rise_ms = float((tt[crossed_90[0]] - tt[crossed_10[0]]) * 1000) \
                if len(crossed_10) and len(crossed_90) else float("nan")

            peak = pp.max() if delta > 0 else pp.min()
            overshoot_pct = float(((peak - p_target) / delta if delta > 0
                                   else (p_target - peak) / (-delta)) * 100)
            overshoot_pct = max(overshoot_pct, 0.0)

            ss = pp[tt >= max(0.0, tt[-1] - 0.5)]
            ss_err_mm = float((ss.mean() - p_target) * 1000)
            # K_eff in commanded-direction = |F_ss / Δ_ss|
            i_F_col = 7 + i
            F_ss = float(a[i_step:, i_F_col][tt >= max(0.0, tt[-1] - 0.5)].mean())
            k_eff = float("nan")
            if ss_err_mm != 0.0:
                k_eff = abs(F_ss / (ss_err_mm / 1000.0))

            out.update(
                step_mm        = float(delta * 1000),
                rise_time_ms   = t_rise_ms,
                overshoot_pct  = overshoot_pct,
                ss_error_mm    = ss_err_mm,
                F_ss_N         = F_ss,
                K_eff_N_per_m  = k_eff,
            )
        return out

    def plot(self, path: Optional[str | Path] = None):
        """Mode-aware figure; saves to ``path`` if given."""
        import matplotlib.pyplot as plt

        a = self.to_array()
        t   = a[:, 0]
        p_d = a[:, 1:4]
        p   = a[:, 4:7]
        F   = a[:, 7:10]

        fig, ax = plt.subplots(2, 1, figsize=(9, 6),
                               gridspec_kw={"height_ratios": [2, 1]})
        fig.suptitle(f"recorder — {self.mode}  ({len(t)} samples, "
                     f"{t[-1]-t[0]:.1f}s)", fontweight="bold")

        labels = "xyz"
        colors_p   = ["tab:blue", "tab:green", "tab:red"]
        colors_p_d = ["#1f77b4aa", "#2ca02caa", "#d62728aa"]

        for i in range(3):
            ax[0].plot(t, (p[:, i] - p[0, i]) * 1000,
                       label=f"p_{labels[i]}", color=colors_p[i], lw=1.0)
            if self.mode in ("osc", "step"):
                ax[0].plot(t, (p_d[:, i] - p[0, i]) * 1000,
                           color=colors_p_d[i], lw=0.9, ls="--",
                           label=f"p_d_{labels[i]}")
        ax[0].set_ylabel("Δ position [mm]")
        ax[0].grid(alpha=0.3)
        ax[0].legend(loc="best", fontsize=8, ncol=3)

        Fmag = np.linalg.norm(F, axis=1)
        ax[1].plot(t, Fmag, color="tab:orange", lw=1.0, label="|F_ext|")
        ax[1].set_xlabel("t [s]")
        ax[1].set_ylabel("|F| [N]")
        ax[1].grid(alpha=0.3)
        ax[1].legend(loc="best", fontsize=8)

        m = self.metrics()
        lines = [f"{k:18s}= {v:.3f}" if isinstance(v, float) else f"{k:18s}= {v}"
                 for k, v in m.items() if k not in ("mode", "n_samples")]
        ax[0].text(0.99, 0.02, "\n".join(lines), transform=ax[0].transAxes,
                   ha="right", va="bottom", family="monospace", fontsize=8,
                   bbox=dict(facecolor="white", alpha=0.7, edgecolor="grey"))

        fig.tight_layout(rect=[0, 0, 1, 0.96])
        if path:
            path = Path(path)
            path.parent.mkdir(parents=True, exist_ok=True)
            fig.savefig(path, dpi=130)
            plt.close(fig)
            return path
        return fig
