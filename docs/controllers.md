# Controllers

The daemon runs a 1 kHz control loop. Streaming `cartesian_impedance` commands flow through two stages before the controller math: a **linear interpolator** between consecutive received targets (bridges any client rate up to 1 kHz), then a 1<sup>st</sup>-order LP smoother (`filter_alpha`, 5%/tick by default).

Tunable defaults for each controller live in `fr3_stack/configs/<name>.yaml`. Profiles (`<name>.<profile>.yaml`) swap at runtime via `Robot(profiles=...)` or `robot.set_profile(...)`.

| Controller            | What it does                                                                 |
| --------------------- | ---------------------------------------------------------------------------- |
| `idle`                | hand-guidable: gravity comp + inertia-aware per-joint damping + optional Coulomb-friction comp |
| `cartesian_impedance` | spring/damper at the EE in base frame     |
| `hybrid`              | per-axis force PID on `n_af` axes + position elsewhere     |



## Idle (hand-guidable)

Kinesthetic-teaching mode. FCI handles gravity; the controller adds three optional layers:

$$
\begin{aligned}
\tau_{\text{damp}}[i] &= -\,d_{\text{rate}}[i] \cdot \bigl(M(q)\,\dot q\bigr)[i] &&\text{(inertia-aware per-joint damping)} \\
\tau &\mathrel{+}=\; \tau_{\text{fric}}(\dot q) &&\text{(Cognetti FR3 sigmoid; default ON)} \\
\tau &\mathrel{+}=\; \tau_{\text{wall}}(\dot q) &&\text{(soft velocity wall near FR3 joint-velocity limits)}
\end{aligned}
$$

Pure zero-torque trips `joint_velocity_violation` within seconds of any hand-guiding push — nothing dissipates the operator's kinetic energy. Idle does **not** hold pose; switch to `cartesian_impedance` for a pose lock.

| Field          | Length | Default                                | Meaning                                                          |
| -------------- | ------ | -------------------------------------- | ---------------------------------------------------------------- |
| `dRate`        | 7      | `[0.3, 1.0, 0.3, 1.0, 0.3, 0.3, 0.3]`  | per-joint velocity decay rate [1/s]; heavier on J2/J4 (gravity sag) |
| `useFriction`  | —      | `true`                                 | cancel Coulomb friction (off = "feel the raw arm")               |

```python
robot.send_idle()                                 # default soft hand-guide
robot.send_idle(d_rate=[1, 3, 1, 3, 1, 1, 1])     # heavier hand
robot.send_idle(use_friction=False)               # raw arm
```

---

## Cartesian impedance

The arm behaves like a 6-DoF spring/damper anchored at a target pose. Task force is mapped to joint torque through $J^{\top}$ (Hogan 1985 impedance, no operational-space inertia weighting).

$$
\begin{aligned}
F_{\text{imp}} &\;=\; K\,(p_d - p) \;-\; D\,\dot x \\[2pt]
\tau_{\text{task}}
  &\;=\; J^{\!\top}\,F_{\text{imp}} \\[4pt]
\tau_{\text{null}}
  &\;=\; N\,\Bigl(K_{\text{null}}\,(q_{\text{null}} - q)\;-\;2\sqrt{K_{\text{null}}}\,\dot q\Bigr),
  \quad
  N = I - J^{\!\top}\bigl(J\,J^{\!\top} + \lambda^{2} I\bigr)^{-1}\,J \\[4pt]
\tau &\;=\; \tau_{\text{task}} + \tau_{\text{null}}
       + C(q,\dot q)
       + J^{\!\top} F_{\text{target}}
       + \tau_{\text{lim}}(q)
       + \mathbb{1}_{\text{use\_friction}}\;\tau_{\text{fric}}(\dot q)
\end{aligned}
$$

Pose error uses the SO(3) logarithm for orientation (rotvec axis·angle, radians), so `K_rot` is honest stiffness in N·m/rad.

### Knobs

| Field            | Length | Default                              | Meaning                                                  |
| ---------------- | ------ | ------------------------------------ | -------------------------------------------------------- |
| `targetPos`      | 3      | —                                    | x, y, z [m] in base frame                                |
| `targetQuatXyzw` | 4      | —                                    | scipy convention                                         |
| `k`              | 6      | `[200, 200, 200, 20, 20, 20]`        | tx ty tz rx ry rz stiffness                              |
| `d`              | 6      | `[28, 28, 28, 9, 9, 9]`              | damping (≈ $2\sqrt{K}$)                                  |
| `qNull`          | 7      | `[0]*7` (sentinel)                   | nullspace anchor; all-zero = snapshot at activation      |
| `kNull`          | —      | 100                                  | nullspace stiffness                                      |
| `filterAlpha`    | —      | 0.05                                 | LP on target (only applied when `ema=true`)              |
| `linearInterp`   | —      | `true`                               | LERP/SLERP between consecutive received cmds             |
| `ema`            | —      | `true`                               | gates the `filterAlpha` LP filter                        |
| `targetWrench`   | 6      | `[0]*6`                              | $F_{\text{target}}$ in base frame                        |
| `maxDelta`       | 6      | `[0]*6`                              | per-axis abs error clip; `0` = no clip                   |
| `useFriction`    | —      | `false`                              | add Stribeck-style comp                                  |

!!! note "Quaternion double-cover"
    The controller flips `R_aligned`'s sign if it dot-products negatively with the smoothed target, so a quaternion and its negative produce identical torque.

### Examples

=== "Stiff in Z"

    ```python
    robot.send_cartesian_impedance(
        target_pos       = [0.5, 0.0, 0.4],
        target_quat_xyzw = [0, 0, 0, 1],
        K = [200, 200, 800, 30, 30, 30],
    )
    ```

=== "Bias the equilibrium with a +5 N feedforward in X"

    ```python
    robot.send_cartesian_impedance(
        target_pos       = pose.pos,
        target_quat_xyzw = pose.quat,
        target_wrench    = [5, 0, 0, 0, 0, 0],
    )
    # Open-loop feedforward, no FT feedback. In free space the spring
    # counters it; the EE settles at Δx ≈ 5 / K_x past target_pos.
    # For closed-loop force tracking use hybrid (n_af ≥ 1).
    ```

=== "Bound the spring"

    ```python
    robot.send_cartesian_impedance(
        target_pos       = pose.pos,
        target_quat_xyzw = pose.quat,
        max_delta        = [0.05, 0.05, 0.05, 0.2, 0.2, 0.2],
    )
    # |error_x| ≤ 5 cm regardless of how far the target jumps
    ```

---

## Hybrid (force/position)

Layered: an inner HFVC loop decomposes the 6-DoF wrench/twist space via a `Tr` matrix into `n_af` force-controlled axes (rows `0..n_af-1`) plus `6 - n_af` velocity-controlled axes. Force axes get a virtual spring + force-tracking PID + damping; velocity axes track pose rigidly. The outer cartesian impedance loop tracks the inner SE(3) target to torque.

Let $S_f \in \mathbb{R}^{6\times 6}$ project onto the first $n_{\text{af}}$ rows, $S_v = I - S_f$.

**Inner (HFVC):**

$$
\begin{aligned}
v_F &\;=\; S_f \cdot M_{\text{adm}}^{-1}\,\bigl(F_{\text{target}}^{Tr} + T_r\,F_{\text{ext}} - D_{\text{adm}}\,v + K_{\text{adm}}\,e\bigr) \\
v_V &\;=\; S_v \cdot K_{\text{adm}}\,e / D_{\text{adm}} \\
\tau_{F\text{-track}} &\;\mathrel{+}=\; \mathrm{PID}\!\bigl(F_{\text{target}}^{Tr} - T_r\,(-F_{\text{ext}})\bigr)
  &&\text{(anti-windup clamp = }\texttt{pidILimit}\text{)} \\
g_{\text{inner}} &\;\leftarrow\; g_{\text{inner}} \;\oplus\; T_r^{-1}\,(v_F + v_V)\,\Delta t
\end{aligned}
$$

**Outer (impedance):**

$$
\tau \;=\; J^{\!\top}\bigl(K\,(g_{\text{inner}} \ominus g_{\text{actual}}) - D\,J\,\dot q\bigr)
       + \tau_{\text{null}} + C(q,\dot q)
$$

`n_af = 0` disables force control on every axis (pure position tracking via the velocity-axis branch). `n_af = 6` is full force control on every axis.

The inner virtual M-K-D (`mAdm` / `kAdm` / `dAdm` below) defines the admittance dynamics on the force-controlled axes; they need a real F/T sensor (see [Quick start](quickstart.md#ft-sensor-optional)).

`target_wrench_Tr` is the wrench the *robot* applies to the environment. To push +5 N along an axis, send `+5`; at steady state the FT sensor reads ≈ `−5` (the reaction).

!!! danger "Don't double-feed"
    Never set `target_wrench` non-zero on `cartesian_impedance` while `hybrid` is active — hybrid already drives the inner FT-PID loop; adding $J^{\top} F$ at the outer layer double-counts the force.

### Knobs

| Field                                           | Length | Default                              | Meaning                                                                |
| ----------------------------------------------- | ------ | ------------------------------------ | ---------------------------------------------------------------------- |
| `nAf`                                           | —      | 0                                    | number of force-controlled axes (0..6)                                 |
| `tr`                                            | 36     | identity                             | 6×6 row-major axis-decomposition matrix (Python accepts `"identity"`)  |
| `targetWrenchTr`                                | 6      | `[0]*6`                              | force command in Tr-space; only first `nAf` entries matter             |
| `mAdm`                                          | 6      | `[5, 5, 5, 0.5, 0.5, 0.5]`           | inner virtual mass                                                     |
| `kAdm`                                          | 6      | `[200, 200, 200, 20, 20, 20]`        | inner virtual stiffness                                                |
| `dAdm`                                          | 6      | `[60, 60, 60, 8, 8, 8]`              | inner virtual damping                                                  |
| `pidPTrans` / `pidITrans` / `pidDTrans`         | —      | 0                                    | scalar trans-block PID gains                                           |
| `pidPRot` / `pidIRot` / `pidDRot`               | —      | 0                                    | scalar rot-block PID gains                                             |
| `pidILimit`                                     | 6      | `[10,10,10,5,5,5]`                   | anti-windup integral clamp                                             |
| `stiction`                                      | 6      | `[0]*6`                              | per-axis dead-band in Tr-space (use stiction *or* PID, not both)       |
| `maxSpringForce` / `maxSpringTorque`            | —      | `50` / `10`                          | spring magnitude clip on force-controlled axes (`0` disables)          |
| `k` / `d` / `qNull` / `kNull` / `useFriction`   |        | same as cart_imp                     | outer-loop impedance                                                   |

### Example — press +5 N along world Z

```python
Tr = np.eye(6)
Tr[[0, 2]] = Tr[[2, 0]]   # row 0 ↔ row 2 → force-controlled axis is world Z
robot.send_hybrid(
    target_pos       = pose.pos,
    target_quat_xyzw = pose.quat,
    n_af             = 1,
    Tr               = Tr.flatten().tolist(),
    target_wrench_Tr = [+5, 0, 0, 0, 0, 0],
)
# FT sensor will read ≈ −5 N in Z (env's reaction).
```

---

## MoveTo (min-jerk setup moves)

`MoveToCmd` is a one-shot "go to pose over T seconds" command. It is **not** a separate controller mode — the daemon selects `cartesian_impedance` and spins up a min-jerk trajectory generator that overrides the pose target each tick until `runTime` elapses, then drops the generator. After that, the controller holds at the goal with the same K/D until the next command.

Use it for resets / setup moves between trials. For closed-loop policy execution, stream `send_cartesian_impedance` directly.

$$
\begin{aligned}
s    &\;=\; t / T \;\in\; [0,1] \\[2pt]
a(s) &\;=\; 10\,s^{3} - 15\,s^{4} + 6\,s^{5}
   &&\text{(5th-order min-jerk: }a,\,\dot a,\,\ddot a = 0\text{ at the endpoints)} \\[2pt]
p(t) &\;=\; p_{0} + a(s)\,(p_{\text{goal}} - p_{0}) \\[2pt]
q(t) &\;=\; \mathrm{slerp}\bigl(q_{0},\, q_{\text{goal}},\, a(s)\bigr)
\end{aligned}
$$

The generator captures $(p_0, q_0)$ from the live pose at activation, so the trajectory always starts where the arm actually is — small delivery delays produce no startup jump.

### Knobs

| Field             | Length | Notes                                                   |
| ----------------- | ------ | ------------------------------------------------------- |
| `targetPos`       | 3      | goal x, y, z [m]                                        |
| `targetQuatXyzw`  | 4      | goal orientation                                        |
| `runTime`         | —      | trajectory duration [s], must be > 0                    |
| `k` / `d`         | 6      | inner cart-imp gains (active during AND after the move) |
| `qNull` / `kNull` | 7 / —  | nullspace anchor                                        |

`filterAlpha` is forced to 1.0 internally — the min-jerk output is already C² smooth.

### Sizing run_time

$$
|v|_{\text{peak}} \approx 1.875\,\frac{\Delta p}{T},
\qquad
|a|_{\text{peak}} \approx 5.77\,\frac{\Delta p}{T^{2}}
$$

Rule of thumb: `T ≥ 1 s` for `Δp ≤ 30 cm`. Round up.

### Lifecycle

| Event                                  | What happens                                                |
| -------------------------------------- | ----------------------------------------------------------- |
| Generator reaches `t = runTime`        | Generator dropped; controller holds at goal                 |
| New `send_cartesian_impedance(...)`    | Generator dropped immediately; new target takes over        |
| New `send_hybrid(...)`                 | Controller switched, generator dropped                      |
| New `send_move_to(...)` mid-trajectory | Old generator dropped; new one starts from live pose        |

```python
robot.move_to(Pose([0.5, 0, 0.45], [0, 0, 0, 1]), run_time=2.0)
```

---

## Streaming interpolation

Bridges any client rate (5 Hz – 1 kHz) up to the daemon's 1 kHz controller tick by **linearly interpolating between the two most recent received targets**. Active for streaming `cartesian_impedance` only (`move_to` has its own min-jerk generator).

Without interpolation, a client at *N* Hz produces an *N* Hz step train at the daemon. The LP smoother blurs the steps but leaves derivative kinks at every boundary, concentrating spectral energy at the client rate — the Jacobian routes that into joint 0 for any horizontal EE motion, producing audible "sawtooth" chatter.

### Algorithm

On each new ZMQ command:

$$
(\text{prev},\,t_{\text{prev}})\;\leftarrow\;(\text{latest},\,t_{\text{latest}}),
\qquad
(\text{latest},\,t_{\text{latest}})\;\leftarrow\;(T_{\text{new}},\,t_{\text{new}})
$$

Each 1 kHz tick:

$$
\begin{aligned}
\alpha &\;=\; \mathrm{clamp}\!\left(\frac{t_{\text{now}} - t_{\text{prev}}}{t_{\text{latest}} - t_{\text{prev}}},\; 0,\; 1\right) \\[4pt]
p_{\text{target}} &\;=\; (1-\alpha)\,p_{\text{prev}} + \alpha\,p_{\text{latest}} \\
q_{\text{target}} &\;=\; \mathrm{slerp}(q_{\text{prev}},\, q_{\text{latest}},\, \alpha)
\end{aligned}
$$

$\alpha$ clamps to `1` past `t_latest`, so the output **holds at the most recent target** if no new command arrives — never extrapolates.

### Interaction with `filter_alpha`

Mostly redundant once the interpolator is in place. The LP still helps when upstream targets themselves are noisy.

| Goal                                          | `filter_alpha`         |
| --------------------------------------------- | ---------------------- |
| Minimum streaming lag (teleop, SpaceMouse)    | `1.0` (pass-through)   |
| Default (smoothing of noisy upstream targets) | `0.05` (≈8 Hz cutoff)  |
| Aggressive smoothing                          | `0.01–0.02`            |

For sparse policy chunks (e.g. diffusion policy at 10 Hz) where you want a specific trajectory shape across all waypoints, use `fr3_stack.InterpolationController` client-side — runs in a subprocess, maintains a `PoseTrajectoryInterpolator` over the full schedule, sends to the daemon at any rate.

---

## Safety helpers

Three terms get summed into $\tau$ regardless of which controller is active.

**Joint-limit repulsion.** Linear ramp from 0 (at 10% from each limit) up to $\pm 10$ N·m at the limit, saturated past it.

**Stribeck-style friction compensation.** $\tau_{\text{fric},i} = f_{p1,i}\,\tanh(f_{p2,i}\,\dot q_i) + f_{p3,i}\,\dot q_i$, off by default per controller via `useFriction`. Defaults are the Cognetti FR3 fit.

**Torque rate limit.** Final stage clamps the per-tick slew to $|\Delta\tau| \le 1\;\text{N·m/ms}$.
