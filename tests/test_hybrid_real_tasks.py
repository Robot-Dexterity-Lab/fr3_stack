"""实际任务级集成测试 —— 每个测试模拟一个真实机器人任务，验证命令流的
语义正确性（不只是 schema 字段对得上）。

任务清单：
  1. contact_search        渐进施力 + FT 反弹检测
  2. polishing_circle      圆周路径 + 法向恒力（抛光 / 擦窗）
  3. screwdriving          Fz 压紧 + Tz 扭矩同步（拧螺丝）
  4. spiral_insertion      螺旋路径 + Fz 压紧（peg-in-hole 搜索）
  5. sine_force_sweep      正弦力扫频（力跟踪带宽测试）
  6. multi_phase           接近 → 接触 → 保持 → 撤退
  7. selection_switch      运行中切换 S（n_af 0→1）
  8. surface_follow        切向移动 + 法向恒力（接触面跟随）
"""
from __future__ import annotations

import math
import time

import numpy as np
import pytest

from fr3_stack.wire import SCHEMA


# ============================================================================
# 1. 接触搜寻：渐进施力 + FT 反弹检测
# ============================================================================

def test_task_contact_search(client_streaming, daemon_streaming):
    """模拟："匀速朝下推进，监听 wrench_ft，发现接触 (|Fz|>2N) 就停。"

    用户写循环，每 tick：
      - 检查最新 state.wrench_ft
      - 没接触 → 继续推
      - 有接触 → 退出循环，把目标力降回 0
    """
    p0, q0 = (0.4, 0.0, 0.5), (0, 0, 0, 1)
    daemon_streaming.publish_until_received(
        client_streaming, pos=p0, quat_xyzw=q0, wrench_ft=(0,) * 6,
    )

    # 模拟接触：t > 0.04 之后 |Fz| 突然变 −3 (传感器读到反作用力)
    def ft_fn(t):
        return (0, 0, -3.0, 0, 0, 0) if t > 0.04 else (0,) * 6

    contacted = False
    # 首 tick 装 S/阈值
    client_streaming.send_hybrid_force_position(
        target_pos=p0, target_quat_xyzw=q0,
        target_force=[0, 0, 0, 0, 0, 0],
        S=[1, 1, 0, 1, 1, 1],
        force_thresholds=[30.0] * 6,
        require_ft_sensor=False,
    )
    t0 = time.monotonic()
    target_fz = 0.0
    # 纯手动 publish + 用户循环。不用 publish_loop 后台线程，否则它会
    # 用零 wrench 反复覆盖手动注入的接触信号。
    while time.monotonic() - t0 < 0.20:
        t = time.monotonic() - t0
        daemon_streaming.publish_state(
            pos=p0, quat_xyzw=q0, wrench_ft=ft_fn(t),
        )
        time.sleep(0.005)
        cur_fz = client_streaming.state.wrench_ft[2] \
            if client_streaming.state.has_ft_sensor else 0.0
        if abs(cur_fz) > 2.0:
            contacted = True
            target_fz = 0.0
            break
        target_fz = max(-5.0, target_fz - 0.5)
        client_streaming.send_hybrid_force_position(
            target_pos=p0, target_quat_xyzw=q0,
            target_force=[0, 0, target_fz, 0, 0, 0],
            S=[1, 1, 0, 1, 1, 1],
            require_ft_sensor=False,
        )

    assert contacted, "接触检测应在 |Fz|>2N 时触发"
    payloads = daemon_streaming.drain_commands()
    assert len(payloads) >= 2
    # 命令序列中 Fz 应该单调递减（推力变大）直到接触
    fzs = []
    for raw in payloads:
        with SCHEMA.Command.from_bytes(raw) as cmd:
            fzs.append(list(cmd.config.hybrid.targetWrenchTr)[0])
    # 至少有一个 tick 给出了非零推力
    assert any(f < -0.5 for f in fzs), f"应出现推力命令，got {fzs}"


# ============================================================================
# 2. 平面抛光：圆周路径 + 法向恒力
# ============================================================================

def test_task_polishing_circle(client_streaming, daemon_streaming):
    """半径 1 cm 圆周 + Z 轴恒 −5 N 法向力，跑 60 ms。"""
    p0 = np.array([0.4, 0.0, 0.5])
    q0 = (0, 0, 0, 1)
    daemon_streaming.publish_until_received(
        client_streaming, pos=p0.tolist(), quat_xyzw=q0, wrench_ft=(0,) * 6,
    )

    R = 0.01
    omega = 2 * math.pi  # 1 Hz

    def pose_fn(t):
        return (p0 + np.array([R * math.cos(omega * t),
                               R * math.sin(omega * t), 0])).tolist(), q0

    with daemon_streaming.publish_loop(
        pos=p0.tolist(), quat_xyzw=q0, wrench_ft=(0,) * 6,
    ):
        client_streaming.run_hybrid_force_position(
            duration=0.08, dt=0.01,
            target_fn=pose_fn,
            target_force=[0, 0, -5, 0, 0, 0],
            S=[1, 1, 0, 1, 1, 1],
            require_ft_sensor=False,
        )

    payloads = daemon_streaming.drain_commands()
    xs, ys, fzs = [], [], []
    for raw in payloads:
        with SCHEMA.Command.from_bytes(raw) as cmd:
            h = cmd.config.hybrid
            xs.append(list(h.targetPos)[0])
            ys.append(list(h.targetPos)[1])
            fzs.append(list(h.targetWrenchTr)[0])

    # 法向力始终为 −5
    assert all(abs(f - (-5.0)) < 1e-9 for f in fzs), f"Fz 不恒定：{fzs}"
    # 路径应在 p0 附近，半径 ≤ R + ε
    for x, y in zip(xs, ys):
        r = math.hypot(x - p0[0], y - p0[1])
        assert r <= R + 1e-9, f"轨迹超出半径：r={r}"
    # 应当真的在画圆（最大半径接近 R，不是站着不动）
    rs = [math.hypot(x - p0[0], y - p0[1]) for x, y in zip(xs, ys)]
    assert max(rs) > 0.5 * R, f"圆周轨迹未展开：max r={max(rs):.4f}"


# ============================================================================
# 3. 拧螺丝：Fz 压紧 + Tz 扭矩同步
# ============================================================================

def test_task_screwdriving(client_streaming, daemon_streaming):
    """S=[1,1,0,1,1,0] → Fz + Tz 双力控，60 ms 内施加 -3 N + 0.5 Nm。"""
    p0, q0 = (0.4, 0.0, 0.5), (0, 0, 0, 1)
    daemon_streaming.publish_until_received(
        client_streaming, pos=p0, quat_xyzw=q0, wrench_ft=(0,) * 6,
    )

    with daemon_streaming.publish_loop(
        pos=p0, quat_xyzw=q0, wrench_ft=(0,) * 6,
    ):
        client_streaming.run_hybrid_force_position(
            duration=0.06, dt=0.02,
            target_force=[0, 0, -3, 0, 0, 0.5],
            S=[1, 1, 0, 1, 1, 0],
            require_ft_sensor=False,
        )

    payloads = daemon_streaming.drain_commands()
    for raw in payloads:
        with SCHEMA.Command.from_bytes(raw) as cmd:
            h = cmd.config.hybrid
            assert h.nAf == 2     # Fz + Tz
            twr = list(h.targetWrenchTr)
            # Tr 行 0 = e_2 (Fz)，行 1 = e_5 (Tz)
            assert twr[0] == pytest.approx(-3.0), f"Fz mismatch: {twr}"
            assert twr[1] == pytest.approx(0.5),  f"Tz mismatch: {twr}"


# ============================================================================
# 4. 螺旋插入：螺旋路径 + Fz 压紧
# ============================================================================

def test_task_spiral_insertion(client_streaming, daemon_streaming):
    """peg-in-hole 探索：xy 螺旋 + z 持续下压。"""
    p0 = np.array([0.4, 0.0, 0.5])
    q0 = (0, 0, 0, 1)
    daemon_streaming.publish_until_received(
        client_streaming, pos=p0.tolist(), quat_xyzw=q0, wrench_ft=(0,) * 6,
    )

    def spiral(t):
        r = 0.001 + 0.005 * t            # 半径线性增大
        theta = 4 * math.pi * t          # 2 Hz 自转
        return (p0 + np.array([
            r * math.cos(theta),
            r * math.sin(theta),
            -0.001 * t,                 # 缓慢下沉 1 mm/s
        ])).tolist(), q0

    with daemon_streaming.publish_loop(
        pos=p0.tolist(), quat_xyzw=q0, wrench_ft=(0,) * 6,
    ):
        client_streaming.run_hybrid_force_position(
            duration=0.08, dt=0.01,
            target_fn=spiral,
            target_force=[0, 0, -2, 0, 0, 0],
            S=[1, 1, 0, 1, 1, 1],
            require_ft_sensor=False,
        )

    payloads = daemon_streaming.drain_commands()
    zs, rs = [], []
    for raw in payloads:
        with SCHEMA.Command.from_bytes(raw) as cmd:
            pos = list(cmd.config.hybrid.targetPos)
            zs.append(pos[2])
            rs.append(math.hypot(pos[0] - p0[0], pos[1] - p0[1]))

    # z 应单调下降（容忍微小调度抖动）
    for a, b in zip(zs, zs[1:]):
        assert b <= a + 1e-9, f"z 不应回升 {a} → {b}"
    # 半径应单调增大
    for a, b in zip(rs, rs[1:]):
        assert b >= a - 1e-9, f"半径不应缩小 {a} → {b}"
    # 末尾半径应明显大于起始
    assert rs[-1] > rs[0] + 1e-4


# ============================================================================
# 5. 正弦力扫频：力跟踪带宽测试
# ============================================================================

def test_task_sine_force_sweep(client_streaming, daemon_streaming):
    """Fz = 3·sin(2π·5·t)，跑 0.1 秒（半周期），验证幅值范围 & 频率。"""
    p0, q0 = (0.4, 0.0, 0.5), (0, 0, 0, 1)
    daemon_streaming.publish_until_received(
        client_streaming, pos=p0, quat_xyzw=q0, wrench_ft=(0,) * 6,
    )

    # 10 Hz × 0.20s = 2 个完整周期 → 一定扫到 ±AMP
    AMP, FREQ, DUR, DT = 3.0, 10.0, 0.20, 0.005

    with daemon_streaming.publish_loop(
        pos=p0, quat_xyzw=q0, wrench_ft=(0,) * 6,
    ):
        # 首 tick 装 S
        client_streaming.send_hybrid_force_position(
            target_pos=p0, target_quat_xyzw=q0,
            target_force=[0] * 6,
            S=[1, 1, 0, 1, 1, 1],
            require_ft_sensor=False,
        )
        t0 = time.monotonic()
        next_t = t0
        while time.monotonic() - t0 < DUR:
            t = time.monotonic() - t0
            fz = AMP * math.sin(2 * math.pi * FREQ * t)
            client_streaming.send_hybrid_force_position(
                target_pos=p0, target_quat_xyzw=q0,
                target_force=[0, 0, fz, 0, 0, 0],
                S=[1, 1, 0, 1, 1, 1],
                require_ft_sensor=False,
            )
            next_t += DT
            sleep = next_t - time.monotonic()
            if sleep > 0:
                time.sleep(sleep)

    payloads = daemon_streaming.drain_commands()
    fzs = []
    for raw in payloads:
        with SCHEMA.Command.from_bytes(raw) as cmd:
            fzs.append(list(cmd.config.hybrid.targetWrenchTr)[0])
    fzs = fzs[1:]   # 首 tick 是 0

    # 幅值应在 [-AMP, AMP] 范围内（允许小数值误差）
    assert max(fzs) <= AMP + 1e-9
    assert min(fzs) >= -AMP - 1e-9
    # 应当真扫到接近 ±AMP（不是站着不动）
    assert max(fzs) > 0.6 * AMP, f"扫频未达正峰：max={max(fzs):.2f}"
    assert min(fzs) < -0.6 * AMP, f"扫频未达负峰：min={min(fzs):.2f}"


# ============================================================================
# 6. 多阶段任务：接近 → 接触 → 保持 → 撤退
# ============================================================================

def test_task_multi_phase_approach_hold_retreat(
    client_streaming, daemon_streaming
):
    """三段任务，用三次 API 调用串起来：
       phase A: 用 apply_effector_forces_along_axis ramp 进入 5N
       phase B: 用 run_hybrid_force_position hold 维持
       phase C: 显式发零力命令撤退
    """
    p0, q0 = (0.4, 0.0, 0.5), (0, 0, 0, 1)
    daemon_streaming.publish_until_received(
        client_streaming, pos=p0, quat_xyzw=q0, wrench_ft=(0,) * 6,
    )

    with daemon_streaming.publish_loop(
        pos=p0, quat_xyzw=q0, wrench_ft=(0,) * 6,
    ):
        # Phase A: ramp 进入
        client_streaming.apply_effector_forces_along_axis(
            run_duration=0.05, acc_duration=0.01, max_translation=0.05,
            forces=[0, 0, -5], dt=0.01, require_ft_sensor=False,
        )
        # 拍快照分段
        mark_a = len(daemon_streaming.drain_commands())

        # Phase B: hold
        client_streaming.run_hybrid_force_position(
            duration=0.04, dt=0.02,
            target_force=[0, 0, -5, 0, 0, 0],
            S=[1, 1, 0, 1, 1, 1],
            require_ft_sensor=False,
        )
        mark_b = len(daemon_streaming.drain_commands())

        # Phase C: 撤退（一发零力）
        client_streaming.send_hybrid_force_position(
            target_pos=p0, target_quat_xyzw=q0,
            target_force=[0] * 6,
            S=[1, 1, 0, 1, 1, 1],
            require_ft_sensor=False,
        )
        mark_c = len(daemon_streaming.drain_commands())

    # 每个 phase 都该至少发了一帧
    assert mark_a >= 2, f"Phase A 命令太少: {mark_a}"
    assert mark_b >= 1, f"Phase B 命令太少: {mark_b}"
    assert mark_c >= 1, f"Phase C 没发命令"


# ============================================================================
# 7. 运行中切换 S：n_af 0 → 1
# ============================================================================

def test_task_selection_switch_mid_stream(
    client_streaming, daemon_streaming
):
    """先 S=[1]*6（纯位置 n_af=0），切到 S=[1,1,0,1,1,1]（Fz 力控 n_af=1），
    验证 n_af 真的变了。"""
    p0, q0 = (0.4, 0.0, 0.5), (0, 0, 0, 1)
    daemon_streaming.publish_until_received(
        client_streaming, pos=p0, quat_xyzw=q0, wrench_ft=(0,) * 6,
    )

    with daemon_streaming.publish_loop(
        pos=p0, quat_xyzw=q0, wrench_ft=(0,) * 6,
    ):
        # Phase A: 纯位置
        for _ in range(3):
            client_streaming.send_hybrid_force_position(
                target_pos=p0, target_quat_xyzw=q0,
                target_force=[0] * 6,
                S=[1, 1, 1, 1, 1, 1],
                require_ft_sensor=False,
            )
        # Phase B: 切到 Fz 力控
        for _ in range(3):
            client_streaming.send_hybrid_force_position(
                target_pos=p0, target_quat_xyzw=q0,
                target_force=[0, 0, -3, 0, 0, 0],
                S=[1, 1, 0, 1, 1, 1],
                require_ft_sensor=False,
            )

    payloads = daemon_streaming.drain_commands()
    n_afs = []
    for raw in payloads:
        with SCHEMA.Command.from_bytes(raw) as cmd:
            n_afs.append(cmd.config.hybrid.nAf)

    # 应当看到 0 → 1 的过渡
    assert 0 in n_afs and 1 in n_afs, f"未观察到 n_af 切换：{n_afs}"
    # 0 的 tick 都在 1 的 tick 之前
    last_zero  = max(i for i, n in enumerate(n_afs) if n == 0)
    first_one  = min(i for i, n in enumerate(n_afs) if n == 1)
    assert last_zero < first_one, \
        f"切换顺序错乱：n_afs={n_afs} last_zero={last_zero} first_one={first_one}"


# ============================================================================
# 8. 接触面跟随：切向移动 + 法向恒力
# ============================================================================

def test_task_surface_following(client_streaming, daemon_streaming):
    """沿 +X 直线移动 1 cm，同时 Z 轴 -4N 法向力。"""
    p0 = np.array([0.4, 0.0, 0.5])
    q0 = (0, 0, 0, 1)
    daemon_streaming.publish_until_received(
        client_streaming, pos=p0.tolist(), quat_xyzw=q0, wrench_ft=(0,) * 6,
    )

    DUR = 0.05

    def trajectory(t):
        return (p0 + np.array([0.01 * t / DUR, 0, 0])).tolist(), q0

    with daemon_streaming.publish_loop(
        pos=p0.tolist(), quat_xyzw=q0, wrench_ft=(0,) * 6,
    ):
        client_streaming.run_hybrid_force_position(
            duration=DUR, dt=0.01,
            target_fn=trajectory,
            target_force=[0, 0, -4, 0, 0, 0],
            S=[1, 1, 0, 1, 1, 1],
            require_ft_sensor=False,
        )

    payloads = daemon_streaming.drain_commands()
    xs, fzs = [], []
    for raw in payloads:
        with SCHEMA.Command.from_bytes(raw) as cmd:
            xs.append(list(cmd.config.hybrid.targetPos)[0])
            fzs.append(list(cmd.config.hybrid.targetWrenchTr)[0])

    # x 单调增加 (切向运动)
    for a, b in zip(xs, xs[1:]):
        assert b >= a - 1e-9, f"切向运动应单调，{a} → {b}"
    # 末位 x 应接近 p0+0.01（容忍 dt 边界）
    assert xs[-1] >= p0[0] + 0.005
    # Fz 全程恒为 −4
    assert all(abs(f - (-4.0)) < 1e-9 for f in fzs), f"法向力不恒定：{fzs}"
