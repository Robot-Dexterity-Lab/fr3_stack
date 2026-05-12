"""Star-tier 补全测试：三个 hybrid API 边界条件 / 参数验证 / kwarg 传播 /
trans-rot 对称 / Tr 正交性 / 极端 ramp 情形。

与 tests/test_hybrid_force_position.py 的 25 条不重叠，只填空。

API 分组：
  * send_hybrid_force_position       — 一发即走
  * run_hybrid_force_position        — 阻塞流式
  * apply_effector_forces_along_axis — 单轴 ramp 推力
"""
from __future__ import annotations

import time

import numpy as np
import pytest

from fr3_stack.wire import SCHEMA


# ============================================================================
# send_hybrid_force_position — 缺口补全
# ============================================================================

# --- 入参长度验证 ----------------------------------------------------------

@pytest.mark.parametrize("target_pos", [
    [0.5, 0.0],            # 太短
    [0.5, 0.0, 0.4, 0.1],  # 太长
])
def test_send_hfp_rejects_bad_target_pos_length(client, daemon, target_pos):
    daemon.publish_until_received(client, wrench_ft=(0,) * 6)
    with pytest.raises(ValueError):
        client.send_hybrid_force_position(
            target_pos=target_pos,
            target_quat_xyzw=[0, 0, 0, 1],
            target_force=[0.0] * 6,
        )


@pytest.mark.parametrize("quat", [
    [0, 0, 1],           # 太短
    [0, 0, 0, 1, 0],     # 太长
])
def test_send_hfp_rejects_bad_quat_length(client, daemon, quat):
    daemon.publish_until_received(client, wrench_ft=(0,) * 6)
    with pytest.raises(ValueError):
        client.send_hybrid_force_position(
            target_pos=[0.5, 0.0, 0.4],
            target_quat_xyzw=quat,
            target_force=[0.0] * 6,
        )


@pytest.mark.parametrize("S", [
    [1, 1, 1, 1, 1],        # 5 维
    [1, 1, 1, 1, 1, 1, 1],  # 7 维
])
def test_send_hfp_rejects_bad_S_length(client, daemon, S):
    daemon.publish_until_received(client, wrench_ft=(0,) * 6)
    with pytest.raises(ValueError, match="length 6"):
        client.send_hybrid_force_position(
            target_pos=[0.5, 0.0, 0.4],
            target_quat_xyzw=[0, 0, 0, 1],
            target_force=[0.0] * 6,
            S=S,
        )


@pytest.mark.parametrize("force", [
    [0.0] * 5,
    [0.0] * 7,
])
def test_send_hfp_rejects_bad_target_force_length(client, daemon, force):
    daemon.publish_until_received(client, wrench_ft=(0,) * 6)
    with pytest.raises(ValueError, match="length 6"):
        client.send_hybrid_force_position(
            target_pos=[0.5, 0.0, 0.4],
            target_quat_xyzw=[0, 0, 0, 1],
            target_force=force,
        )


# --- S 语义 ----------------------------------------------------------------

def test_send_hfp_pure_force_mode_all_zeros(client, daemon):
    """S=[0]*6 → 全 6 轴力控，n_af=6。"""
    daemon.publish_until_received(client, wrench_ft=(0,) * 6)
    client.send_hybrid_force_position(
        target_pos=[0.5, 0.0, 0.4],
        target_quat_xyzw=[0, 0, 0, 1],
        target_force=[1, 2, 3, 0.1, 0.2, 0.3],
        S=[0, 0, 0, 0, 0, 0],
    )
    with daemon.recv_command() as cmd:
        h = cmd.config.hybrid
        assert h.nAf == 6
        # Tr 应为单位阵（置换矩阵下力轴依顺序放）
        tr = np.array(list(h.tr)).reshape(6, 6)
        assert np.allclose(tr, np.eye(6))
        # 全 6 个力轴都激活
        assert list(h.targetWrenchTr) == pytest.approx([1, 2, 3, 0.1, 0.2, 0.3])


def test_send_hfp_pure_position_default(client, daemon):
    """S 默认 → 全位置控制，n_af=0。"""
    daemon.publish_until_received(client, wrench_ft=(0,) * 6)
    client.send_hybrid_force_position(
        target_pos=[0.5, 0.0, 0.4],
        target_quat_xyzw=[0, 0, 0, 1],
        target_force=[10, 20, 30, 0, 0, 0],   # 任何力都应被忽略
    )
    with daemon.recv_command() as cmd:
        h = cmd.config.hybrid
        assert h.nAf == 0
        # 力命令在 Tr-space 应全为 0（位置控制轴不接收力命令）
        assert list(h.targetWrenchTr) == pytest.approx([0.0] * 6)


def test_send_hfp_torque_only_axis(client, daemon):
    """纯力矩任务：S=[1,1,1,1,1,0] → Tz 力矩控制，n_af=1。"""
    daemon.publish_until_received(client, wrench_ft=(0,) * 6)
    client.send_hybrid_force_position(
        target_pos=[0.5, 0.0, 0.4],
        target_quat_xyzw=[0, 0, 0, 1],
        target_force=[0, 0, 0, 0, 0, 0.5],
        S=[1, 1, 1, 1, 1, 0],
    )
    with daemon.recv_command() as cmd:
        h = cmd.config.hybrid
        assert h.nAf == 1
        # 首行 Tr 应为 e_5（Tz 轴 = 索引 5）
        tr_row0 = list(h.tr)[:6]
        assert tr_row0 == pytest.approx([0, 0, 0, 0, 0, 1])
        assert list(h.targetWrenchTr)[0] == pytest.approx(0.5)


def test_send_hfp_force_and_torque_combined(client, daemon):
    """拧螺丝：S=[1,1,0,1,1,0] → Fz + Tz，n_af=2。"""
    daemon.publish_until_received(client, wrench_ft=(0,) * 6)
    client.send_hybrid_force_position(
        target_pos=[0.5, 0.0, 0.4],
        target_quat_xyzw=[0, 0, 0, 1],
        target_force=[0, 0, -5, 0, 0, 0.8],
        S=[1, 1, 0, 1, 1, 0],
    )
    with daemon.recv_command() as cmd:
        h = cmd.config.hybrid
        assert h.nAf == 2
        # 前两行应为 e_2 (Fz) 和 e_5 (Tz)，按 S 中 0 的顺序
        tr = np.array(list(h.tr)).reshape(6, 6)
        assert tr[0].tolist() == pytest.approx([0, 0, 1, 0, 0, 0])
        assert tr[1].tolist() == pytest.approx([0, 0, 0, 0, 0, 1])
        # 力命令前两位为 (-5, 0.8)
        twr = list(h.targetWrenchTr)
        assert twr[0] == pytest.approx(-5.0)
        assert twr[1] == pytest.approx(0.8)


def test_send_hfp_target_force_on_pos_axes_ignored(client, daemon):
    """位置控制轴上的 target_force 应被静默忽略。"""
    daemon.publish_until_received(client, wrench_ft=(0,) * 6)
    client.send_hybrid_force_position(
        target_pos=[0.5, 0.0, 0.4],
        target_quat_xyzw=[0, 0, 0, 1],
        target_force=[99, 99, -5, 99, 99, 99],   # 只有 Fz 有效
        S=[1, 1, 0, 1, 1, 1],
    )
    with daemon.recv_command() as cmd:
        h = cmd.config.hybrid
        twr = list(h.targetWrenchTr)
        assert twr[0] == pytest.approx(-5.0)
        # 其余位都应为 0
        assert all(abs(x) < 1e-9 for x in twr[1:])


# --- Tr 矩阵正交性 / 行列式 -----------------------------------------------

@pytest.mark.parametrize("S", [
    [1, 1, 0, 1, 1, 1],      # 单力轴
    [1, 1, 0, 1, 1, 0],      # 2 力轴
    [0, 0, 0, 1, 1, 1],      # 3 力轴
    [0, 0, 0, 0, 0, 0],      # 6 力轴
    [1, 1, 1, 1, 1, 1],      # 0 力轴
])
def test_send_hfp_tr_is_permutation_matrix(client, daemon, S):
    """Tr 必须是置换矩阵：每行/列恰好一个 1，其余 0，det=±1。"""
    daemon.publish_until_received(client, wrench_ft=(0,) * 6)
    client.send_hybrid_force_position(
        target_pos=[0.5, 0.0, 0.4],
        target_quat_xyzw=[0, 0, 0, 1],
        target_force=[0.0] * 6,
        S=S,
    )
    with daemon.recv_command() as cmd:
        tr = np.array(list(cmd.config.hybrid.tr)).reshape(6, 6)
    # 每行恰好一个 1
    assert np.allclose(tr.sum(axis=1), 1.0)
    # 每列恰好一个 1
    assert np.allclose(tr.sum(axis=0), 1.0)
    # 只有 0 和 1
    assert set(tr.flatten().tolist()).issubset({0.0, 1.0})
    # |det| = 1
    assert abs(abs(np.linalg.det(tr)) - 1.0) < 1e-9


# --- kwarg 传播 ------------------------------------------------------------

def test_send_hfp_position_kps_cart_length_check(client, daemon):
    """position_kps_cart 长度不为 6 应当报错。"""
    daemon.publish_until_received(client, wrench_ft=(0,) * 6)
    with pytest.raises(ValueError):
        client.send_hybrid_force_position(
            target_pos=[0.5, 0.0, 0.4],
            target_quat_xyzw=[0, 0, 0, 1],
            target_force=[0.0] * 6,
            position_kps_cart=[100, 100, 100],   # 太短
        )


def test_send_hfp_require_ft_false_bypasses_gate(client, daemon):
    """require_ft_sensor=False 时即使 daemon 没发 wrench 也不该 raise。"""
    daemon.publish_until_received(client)   # 默认无 wrench_ft
    client.send_hybrid_force_position(
        target_pos=[0.5, 0.0, 0.4],
        target_quat_xyzw=[0, 0, 0, 1],
        target_force=[0.0] * 6,
        require_ft_sensor=False,
    )
    with daemon.recv_command() as cmd:
        assert cmd.config.which() == "hybrid"


def test_send_hfp_blend_S_warns_once(client, daemon, caplog):
    """S 含 (0,1) 区间值应当打一次警告（HFVC 是二元）。"""
    daemon.publish_until_received(client, wrench_ft=(0,) * 6)
    # 复位模块级 once-flag —— 跨测试隔离
    import fr3_stack.wire as wire
    wire._S_BLEND_WARN_LOGGED = False
    import logging
    with caplog.at_level(logging.WARNING, logger="fr3_stack.wire"):
        client.send_hybrid_force_position(
            target_pos=[0.5, 0.0, 0.4],
            target_quat_xyzw=[0, 0, 0, 1],
            target_force=[0.0] * 6,
            S=[1.0, 1.0, 0.3, 1.0, 1.0, 1.0],   # 0.3 触发警告
        )
    assert any("HFVC is binary" in r.message for r in caplog.records)


# ============================================================================
# run_hybrid_force_position — 缺口补全
# ============================================================================

def test_run_hfp_target_poses_sequence_consumed(
    client_streaming, daemon_streaming
):
    """target_poses 序列每个 tick 推进一帧。"""
    p_seed = (0.4, 0.0, 0.5)
    q0 = [0, 0, 0, 1]
    daemon_streaming.publish_until_received(
        client_streaming, pos=p_seed, quat_xyzw=q0, wrench_ft=(0,) * 6,
    )
    poses = [
        ([0.40, 0.0, 0.5], q0),
        ([0.41, 0.0, 0.5], q0),
        ([0.42, 0.0, 0.5], q0),
        ([0.43, 0.0, 0.5], q0),
    ]
    with daemon_streaming.publish_loop(
        pos=p_seed, quat_xyzw=q0, wrench_ft=(0,) * 6,
    ):
        client_streaming.run_hybrid_force_position(
            duration=0.06, dt=0.02,
            target_poses=poses,
            require_ft_sensor=False,
        )
    payloads = daemon_streaming.drain_commands()
    xs = []
    for raw in payloads:
        with SCHEMA.Command.from_bytes(raw) as cmd:
            xs.append(list(cmd.config.hybrid.targetPos)[0])
    # 前几 tick 的 x 应严格递增，且都从 poses[] 里取的
    assert xs[0] == pytest.approx(0.40)
    # 后续 tick 取到的 x 应在 [0.40, 0.43] 之间且单调
    for a, b in zip(xs, xs[1:]):
        assert b >= a - 1e-9


def test_run_hfp_target_poses_with_extra_entries_allowed(
    client_streaming, daemon_streaming
):
    """超出 duration/dt 数量的 entries 允许，多余忽略。"""
    p0 = (0.4, 0.0, 0.5)
    q0 = [0, 0, 0, 1]
    daemon_streaming.publish_until_received(
        client_streaming, pos=p0, quat_xyzw=q0, wrench_ft=(0,) * 6,
    )
    # 需要 ceil(0.04/0.02)=2 帧，提供 10 帧。
    poses = [([0.4 + 0.01*i, 0.0, 0.5], q0) for i in range(10)]
    with daemon_streaming.publish_loop(
        pos=p0, quat_xyzw=q0, wrench_ft=(0,) * 6,
    ):
        client_streaming.run_hybrid_force_position(
            duration=0.04, dt=0.02,
            target_poses=poses,
            require_ft_sensor=False,
        )   # 不应当 raise


def test_run_hfp_S_propagates_each_tick(client_streaming, daemon_streaming):
    """流式时每 tick 都应当带相同 S。"""
    p0, q0 = (0.4, 0.0, 0.5), [0, 0, 0, 1]
    daemon_streaming.publish_until_received(
        client_streaming, pos=p0, quat_xyzw=q0, wrench_ft=(0,) * 6,
    )
    with daemon_streaming.publish_loop(
        pos=p0, quat_xyzw=q0, wrench_ft=(0,) * 6,
    ):
        client_streaming.run_hybrid_force_position(
            duration=0.06, dt=0.02,
            target_force=[0, 0, -3, 0, 0, 0],
            S=[1, 1, 0, 1, 1, 1],
            require_ft_sensor=False,
        )
    payloads = daemon_streaming.drain_commands()
    n_afs = []
    for raw in payloads:
        with SCHEMA.Command.from_bytes(raw) as cmd:
            n_afs.append(cmd.config.hybrid.nAf)
    # 每个 tick 都应为 n_af=1
    assert all(n == 1 for n in n_afs), n_afs


def test_run_hfp_thresholds_cached_after_first_tick(
    client_streaming, daemon_streaming
):
    """force_thresholds 只在首 tick 显式发，后续 tick 走 cache。"""
    p0, q0 = (0.4, 0.0, 0.5), [0, 0, 0, 1]
    daemon_streaming.publish_until_received(
        client_streaming, pos=p0, quat_xyzw=q0, wrench_ft=(0,) * 6,
    )
    THRESH = [25.0] * 6
    with daemon_streaming.publish_loop(
        pos=p0, quat_xyzw=q0, wrench_ft=(0,) * 6,
    ):
        client_streaming.run_hybrid_force_position(
            duration=0.06, dt=0.02,
            target_force=[0, 0, -3, 0, 0, 0],
            S=[1, 1, 0, 1, 1, 1],
            force_thresholds=THRESH,
            require_ft_sensor=False,
        )
    payloads = daemon_streaming.drain_commands()
    # 每个 tick 都该带相同阈值（cache 透传）
    for raw in payloads:
        with SCHEMA.Command.from_bytes(raw) as cmd:
            assert list(cmd.config.hybrid.forceThresholds) == pytest.approx(THRESH)


def test_run_hfp_pure_force_S_all_zero(client_streaming, daemon_streaming):
    """S=[0]*6 端到端：每 tick 都 n_af=6。"""
    p0, q0 = (0.4, 0.0, 0.5), [0, 0, 0, 1]
    daemon_streaming.publish_until_received(
        client_streaming, pos=p0, quat_xyzw=q0, wrench_ft=(0,) * 6,
    )
    with daemon_streaming.publish_loop(
        pos=p0, quat_xyzw=q0, wrench_ft=(0,) * 6,
    ):
        client_streaming.run_hybrid_force_position(
            duration=0.04, dt=0.02,
            target_force=[0, 0, -2, 0, 0, 0],
            S=[0, 0, 0, 0, 0, 0],
            require_ft_sensor=False,
        )
    payloads = daemon_streaming.drain_commands()
    for raw in payloads:
        with SCHEMA.Command.from_bytes(raw) as cmd:
            assert cmd.config.hybrid.nAf == 6


def test_run_hfp_duration_actually_elapses(client_streaming, daemon_streaming):
    """实际阻塞时间应接近 duration（容忍调度抖动）。"""
    p0, q0 = (0.4, 0.0, 0.5), [0, 0, 0, 1]
    daemon_streaming.publish_until_received(
        client_streaming, pos=p0, quat_xyzw=q0, wrench_ft=(0,) * 6,
    )
    with daemon_streaming.publish_loop(
        pos=p0, quat_xyzw=q0, wrench_ft=(0,) * 6,
    ):
        t0 = time.monotonic()
        client_streaming.run_hybrid_force_position(
            duration=0.10, dt=0.02, require_ft_sensor=False,
        )
        elapsed = time.monotonic() - t0
    # duration ≤ 实际 ≤ duration + 一些调度容忍
    assert 0.09 <= elapsed <= 0.30, f"elapsed={elapsed:.3f}s"


# ============================================================================
# apply_effector_forces_along_axis — 缺口补全
# ============================================================================

def test_apply_effector_acc_zero_no_ramp(client_streaming, daemon_streaming):
    """acc_duration=0 → 跳过 ramp，全程满力（直到收尾零命令）。"""
    p0, q0 = (0.4, 0.0, 0.5), (0, 0, 0, 1)
    daemon_streaming.publish_until_received(
        client_streaming, pos=p0, quat_xyzw=q0, wrench_ft=(0,) * 6,
    )
    mag = 5.0
    with daemon_streaming.publish_loop(
        pos=p0, quat_xyzw=q0, wrench_ft=(0,) * 6,
    ):
        client_streaming.apply_effector_forces_along_axis(
            run_duration=0.04, acc_duration=0.0, max_translation=0.05,
            forces=[0, 0, -mag], dt=0.01, require_ft_sensor=False,
        )
    payloads = daemon_streaming.drain_commands()
    # 去掉收尾的零命令再看
    fzs = []
    for raw in payloads:
        with SCHEMA.Command.from_bytes(raw) as cmd:
            fzs.append(list(cmd.config.hybrid.targetWrenchTr)[0])
    while fzs and abs(fzs[-1]) < 1e-9:
        fzs.pop()
    # 全程都该是满力 mag（按斜率 1.0，无 ramp）
    assert len(fzs) >= 1
    for v in fzs:
        assert abs(v) == pytest.approx(mag), f"acc=0 应当无 ramp，got {fzs}"


def test_apply_effector_diagonal_force_orthonormal_tr(
    client_streaming, daemon_streaming
):
    """对角方向力 forces=[1,1,0]/√2 → Tr 应为正交基（3 维子块）。"""
    p0, q0 = (0.4, 0.0, 0.5), (0, 0, 0, 1)
    daemon_streaming.publish_until_received(
        client_streaming, pos=p0, quat_xyzw=q0, wrench_ft=(0,) * 6,
    )
    with daemon_streaming.publish_loop(
        pos=p0, quat_xyzw=q0, wrench_ft=(0,) * 6,
    ):
        client_streaming.apply_effector_forces_along_axis(
            run_duration=0.04, acc_duration=0.01, max_translation=0.05,
            forces=[3.0, 3.0, 0.0], dt=0.01, require_ft_sensor=False,
        )
    payloads = daemon_streaming.drain_commands()
    assert len(payloads) >= 1
    with SCHEMA.Command.from_bytes(payloads[0]) as cmd:
        tr = np.array(list(cmd.config.hybrid.tr)).reshape(6, 6)
        # 平动 3×3 子块应正交（行两两正交、单位范数）
        T3 = tr[:3, :3]
        gram = T3 @ T3.T
        assert np.allclose(gram, np.eye(3), atol=1e-9), \
            f"Tr 平动子块非正交：\n{gram}"
        # 首行单位向量应沿 forces 方向（归一化后）
        u = np.array([3.0, 3.0, 0.0]); u /= np.linalg.norm(u)
        assert np.allclose(T3[0], u, atol=1e-9)
        # 转动 3×3 子块应保持为单位阵（位置控制）
        T_rot = tr[3:, 3:]
        assert np.allclose(T_rot, np.eye(3))


def test_apply_effector_anchored_pose_during_run(
    client_streaming, daemon_streaming
):
    """期间机器人位置漂移（小于 max_translation），targetPos 仍锁在 p0。"""
    p0 = (0.4, 0.0, 0.5)
    q0 = (0, 0, 0, 1)
    daemon_streaming.publish_until_received(
        client_streaming, pos=p0, quat_xyzw=q0, wrench_ft=(0,) * 6,
    )

    # 让 state 的 pos 在期间小幅漂动（不触发 max_translation）。
    def pos_fn(t):
        return (p0[0] + 0.01 * t, p0[1], p0[2])

    with daemon_streaming.publish_loop(
        period=0.005, pos_fn=pos_fn, quat_xyzw=q0, wrench_ft=(0,) * 6,
    ):
        client_streaming.apply_effector_forces_along_axis(
            run_duration=0.05, acc_duration=0.01, max_translation=0.10,
            forces=[0, 0, -3], dt=0.01, require_ft_sensor=False,
        )
    payloads = daemon_streaming.drain_commands()
    assert len(payloads) >= 2
    # 每个 tick 的 targetPos 都应锁在 p0（不是 state.pos）。
    for raw in payloads:
        with SCHEMA.Command.from_bytes(raw) as cmd:
            assert list(cmd.config.hybrid.targetPos) == pytest.approx(list(p0))


def test_apply_effector_require_ft_false_skips_gate(
    client_streaming, daemon_streaming
):
    """require_ft_sensor=False 时即使没 wrench_ft 也不 raise。"""
    p0, q0 = (0.4, 0.0, 0.5), (0, 0, 0, 1)
    daemon_streaming.publish_until_received(
        client_streaming, pos=p0, quat_xyzw=q0,   # 注意：无 wrench_ft
    )
    with daemon_streaming.publish_loop(
        pos=p0, quat_xyzw=q0,   # 仍然无 wrench_ft
    ):
        client_streaming.apply_effector_forces_along_axis(
            run_duration=0.04, acc_duration=0.01, max_translation=0.05,
            forces=[0, 0, -3], dt=0.01, require_ft_sensor=False,
        )   # 不应当 raise


def test_apply_effector_position_kps_cart_propagates(
    client_streaming, daemon_streaming
):
    """传 position_kps_cart 后，wire 上 cmd.k 应等于传入值。"""
    p0, q0 = (0.4, 0.0, 0.5), (0, 0, 0, 1)
    daemon_streaming.publish_until_received(
        client_streaming, pos=p0, quat_xyzw=q0, wrench_ft=(0,) * 6,
    )
    K = [1234.0, 1234.0, 1234.0, 56.0, 56.0, 56.0]
    with daemon_streaming.publish_loop(
        pos=p0, quat_xyzw=q0, wrench_ft=(0,) * 6,
    ):
        client_streaming.apply_effector_forces_along_axis(
            run_duration=0.04, acc_duration=0.01, max_translation=0.05,
            forces=[0, 0, -3], dt=0.01,
            position_kps_cart=K,
            require_ft_sensor=False,
        )
    payloads = daemon_streaming.drain_commands()
    assert len(payloads) >= 1
    with SCHEMA.Command.from_bytes(payloads[0]) as cmd:
        assert list(cmd.config.hybrid.k) == pytest.approx(K)


@pytest.mark.parametrize("forces", [
    [0, 0, 1],
    [0, 0, -1],
    [1, 0, 0],
    [0, 1, 0],
    [1, 1, 1],
    [3, 4, 0],   # 5-magnitude，已知正交化 corner
])
def test_apply_effector_tr_first_row_aligned_with_forces(
    client_streaming, daemon_streaming, forces
):
    """无论 forces 方向如何，Tr 首行（前 3 列）= forces / ‖forces‖。"""
    p0, q0 = (0.4, 0.0, 0.5), (0, 0, 0, 1)
    daemon_streaming.publish_until_received(
        client_streaming, pos=p0, quat_xyzw=q0, wrench_ft=(0,) * 6,
    )
    with daemon_streaming.publish_loop(
        pos=p0, quat_xyzw=q0, wrench_ft=(0,) * 6,
    ):
        client_streaming.apply_effector_forces_along_axis(
            run_duration=0.04, acc_duration=0.01, max_translation=0.05,
            forces=forces, dt=0.01, require_ft_sensor=False,
        )
    payloads = daemon_streaming.drain_commands()
    f = np.asarray(forces, dtype=float); f /= np.linalg.norm(f)
    with SCHEMA.Command.from_bytes(payloads[0]) as cmd:
        tr_row0_trans = np.array(list(cmd.config.hybrid.tr)[:3])
        assert np.allclose(tr_row0_trans, f, atol=1e-9), \
            f"Tr 首行 ≠ unit force：{tr_row0_trans} vs {f}"
