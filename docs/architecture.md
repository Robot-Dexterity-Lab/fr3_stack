# Architecture

Two processes plus the contract between them.

```
┌─────────────────────────┐        ZMQ (CONFLATE=1)         ┌──────────────────────────┐
│   Workstation (Python)  │   ─── Cap'n Proto Command ──→   │     NUC (C++ daemon)     │
│                         │                                 │                          │
│   fr3_stack.Robot       │   ←── Cap'n Proto State    ───  │   libfranka @ 1 kHz RT   │
│   fr3_stack.Arm         │                                 │   chosen controller      │
└─────────────────────────┘                                 └──────────────────────────┘
```

- **Daemon (C++)** runs on the FR3 NUC. Owns libfranka's 1 kHz real-time control loop, the optional Bota EtherCAT F/T worker, and two ZMQ sockets (command in, state out).
- **Client (Python)** runs on a workstation. Sends commands; reads state at ~200 Hz.
- **Wire** is `proto/fr3.capnp` — included by both sides at build/import time. Edits propagate to both.

Both ZMQ sockets use `CONFLATE=1` (latest-wins), so a slow consumer never backs up the RT loop and a slow producer never freezes the controller.

## Repo layout

```
fr3-stack/
├── proto/fr3.capnp                       # wire contract
├── include/fr3_stack/                    # C++ headers
│   ├── controllers/                      #   one .hpp per controller
│   ├── utils/                            #   RT-safe inline helpers
│   ├── sensors/                          #   WrenchSource + Bota + payload-calib decorator
│   └── motion_generator.hpp              #   min-jerk planner (MoveTo)
├── src/                                  # C++ implementations
│   ├── main.cpp                          #   daemon entry — ZMQ + RT loop
│   ├── controllers/*.cpp                 #   controller bodies → fr3_controllers lib
│   ├── bin/*.cpp                         #   standalone demo binaries
│   └── sensors/                          #   Bota glue + payload_calib.cpp
├── fr3_stack/                            # Python client package
│   ├── robot.py                          #   streaming wire (Robot, send_*)
│   ├── client.py                         #   Arm — pose-centric facade
│   ├── configs/*.yaml                    #   per-controller default gains
│   └── sensors/bota/                     #   FT calibration + publish tools
├── tests/                                # pytest + g++ math mock
└── examples/                             # smoke tests on real FR3
```

## Two API layers

| Class   | Module               | Role                                                                            |
| ------- | -------------------- | ------------------------------------------------------------------------------- |
| `Robot` | `fr3_stack.robot`    | Streaming wire — one `send_*` per command. Direct access to every wire field.   |
| `Arm`   | `fr3_stack.client`   | Pose-centric facade for inference / teleop loops. Composes a `Robot`; `arm.robot` is the escape hatch. |

`Arm` exposes the small surface inference scripts actually use (`observe / send / move_to / hold / set_stiffness / use_profile`). Anything richer — explicit hybrid wrenches, raw nullspace tuning — drops through to `arm.robot`.

## RT loop, in one paragraph

`src/main.cpp` opens the libfranka control channel and calls `robot.control(callback)` with a 1 kHz callback. Each tick the callback (a) grabs the latest command from a lock-free queue fed by the ZMQ receive thread, (b) calls the active controller's `update(state)` to produce `τ`, (c) returns `τ` to libfranka. A separate state-publisher thread reads the cached `RobotState` and emits `State` messages at ~200 Hz. Switching controllers calls `reset(state)` on the new one so it re-anchors at the live pose — no jump.

## Conventions

- **Wire strings are stable.** The capnp union arm names are the contract — internal C++ class names can move freely.
- **`include/` = declarations; `src/` = implementations.** Templates and small inline helpers in headers are the exceptions.
- **Templated, RT-safe utilities.** Anything called inside the libfranka callback uses fixed-size Eigen and templates — no heap allocations per tick.
- **Torque rate limit.** The final stage clamps $|\Delta\tau| \le 1\;\text{N·m/ms}$ regardless of which controller is active, so target steps and controller switches never hit the motor driver as raw discontinuities.
