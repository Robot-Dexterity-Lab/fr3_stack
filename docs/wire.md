# Wire protocol

Cap'n Proto schema lives in [`proto/fr3.capnp`](https://github.com/kingchou007/fr3_stack/blob/main/proto/fr3.capnp) — the single source of truth, shared by:

- the Python client (`pycapnp` parses at import time)
- the C++ daemon (`capnp_generate_cpp` at build time)

## Vector ordering

| Vector            | Order                                  |
| ----------------- | -------------------------------------- |
| `pos`             | x, y, z                                |
| `quatXyzw`        | x, y, z, w  (scipy convention)         |
| `k` / `d`         | tx, ty, tz, rx, ry, rz                 |
| `q`               | j1 .. j7                               |

## Schema

```capnp
struct Command {
    termination     @0 :Bool;
    config :union {
        idle               @1 :IdleCmd;
        cartesianImpedance @2 :CartesianImpedanceCmd;
        hybrid             @5 :HybridCmd;
        moveTo             @6 :MoveToCmd;
    }
}

struct State {
    controller     @0  :Text;
    pos            @1  :List(Float64);   # length 3
    quatXyzw       @2  :List(Float64);   # length 4
    q              @3  :List(Float64);   # length 7
    dq             @4  :List(Float64);   # length 7
    wrenchExt      @5  :List(Float64);   # length 6  (libfranka O_F_ext_hat_K, base frame)
    timestamp      @6  :Float64;
    running        @7  :Bool;
    lastError      @8  :Text;
    wrenchFt       @9  :List(Float64);   # length 6 OR empty (compensated when ftCompensated)
    wrenchFtRaw    @10 :List(Float64);   # length 6 OR empty (always raw, only sensor tare)
    ftCompensated  @11 :Bool;            # daemon is subtracting payload gravity + bias
}
```

Each `Command` is a *complete* config — clients send the full struct each time. The Python client caches the last-sent values per controller so partial updates from user code still produce a fully-populated message on the wire.

The full per-command structs (`IdleCmd`, `CartesianImpedanceCmd`, `HybridCmd`, `MoveToCmd`) live in [`proto/fr3.capnp`](https://github.com/kingchou007/fr3_stack/blob/main/proto/fr3.capnp).

### `IdleCmd` semantics

```capnp
struct IdleCmd {
    dRate        @0 :List(Float64);   # length 7, per-joint damping rate [1/s]
    useFriction  @1 :Bool;            # cancel joint friction (Cognetti FR3 fit)
}
```

See [Controllers → Idle](controllers.md#idle-hand-guidable) for the dynamics and tuning notes.

### `CartesianImpedanceCmd` smoothing toggles

`linearInterp` (LERP/SLERP between consecutive received targets in the daemon dispatch) and `ema` (the controller's 1<sup>st</sup>-order LP) are both wire-level booleans, default `true`. The recommended combo for ≤5 Hz target streams is `linearInterp=true, ema=false` — see [Streaming interpolation](controllers.md#streaming-interpolation).

### `wrenchFt` / `wrenchFtRaw` / `ftCompensated`

The daemon publishes `wrenchFt` (length-6, base frame, rotated from sensor frame via `R_O_EE`) iff `--ft-sensor-kind` is set AND the backend's worker has produced at least one frame; otherwise it sends an empty list. The Python client maps empty → `State.wrench_ft = None`, exposed as `State.has_ft_sensor`.

When the daemon also loaded a payload calibration YAML at boot, it wraps the sensor source in a `CompensatedWrenchSource` decorator:

| Field           | Calib loaded               | No calib            | No FT backend |
| --------------- | -------------------------- | ------------------- | ------------- |
| `wrenchFt`      | gravity + bias subtracted  | raw (only tare)     | empty list    |
| `wrenchFtRaw`   | always raw                 | mirrors `wrenchFt`  | empty list    |
| `ftCompensated` | `true`                     | `false`             | `false`       |

### `HybridCmd` highlights

- `tr` is row-major 6×6 (length 36). The Python client accepts the literal `"identity"`, a 2D matrix, or a length-36 sequence.
- `targetWrenchTr` is in Tr-space; only the first `nAf` entries are active.
- Anti-windup integral clamp `pidILimit` is per-axis (length 6).
- See [Controllers → Hybrid](controllers.md#hybrid-forceposition).

### `MoveToCmd` semantics

Daemon-side min-jerk generator anchored at the live pose. While `runTime` elapses the generator's output replaces the cartesian-impedance target every tick; afterwards the controller holds at the goal with the supplied K/D. Use it for resets and setup moves — closed-loop policy streams should send `cartesianImpedance` directly.

`MoveToCmd` is **not** a separate controller mode: the daemon selects `cartesian_impedance` internally, so `state.controller` reads `"cartesian_impedance"` while the move plays. `filterAlpha` is forced to 1.0 internally. See [Controllers → MoveTo](controllers.md#moveto-min-jerk-setup-moves) for the math, knobs, and lifecycle.

## Streaming semantics — latest-wins, never blocks

Every `send_*` returns immediately and supersedes any in-flight command. The RT loop never waits on the network.

| Scenario                      | What happens                                          | What to do                                   |
| ----------------------------- | ----------------------------------------------------- | -------------------------------------------- |
| Python streams at 1000 Hz     | wire conflates, RT sees latest                        | nothing                                      |
| target jumps 50 cm            | filter blends, ~200 ms to 90 %                        | raise `filter_alpha` to 0.1–0.2 if too soft  |
| Python stalls for seconds     | RT holds last target (impedance still live)           | add a watchdog if you want auto-idle         |
| `send_*` while `moveTo` active| generator dropped immediately, new command takes over | nothing                                      |
| Cancel current target         | send a new target, or `send_idle()`                   | no explicit "cancel"                         |

## Debugging a payload

```bash
nc -l 5555 | capnp decode proto/fr3.capnp Command
```
