# Collaborative development

Write all documentation in English. Work against
`https://github.com/Robot-Dexterity-Lab/fr3_stack.git`.

## Module ownership and boundaries

- `fr3_stack/dual_arm.py`: transport-independent coordination, paired validation,
  state transitions, common loop timing, and fault propagation. Depend on
  `ArmEndpoint`, not ZMQ, Cap'n Proto, Robot internals, or C++ controllers.
- `fr3_stack/dual_arm_robot.py`: the adapter to the single-arm public Robot API.
  Keep transport-specific behavior here; require an explicit stop policy.
- `fr3_stack/robot.py` and `state.py`: single-arm transport and observations.
  Keep these usable independently of dual-arm orchestration.
- `proto/fr3.capnp`, `src/main.cpp`: wire contract and daemon dispatch.
  Preserve field ordinals and backward compatibility when extending the protocol.
- `src/controllers/`, `include/fr3_stack/controllers/`: low-level control math.
  Do not introduce workstation orchestration into the 1 kHz controller path.

## Coordinating changes

- Agree on interface, state, units, clock domain, and failure semantics before
  changing a shared boundary. Record these in the PR, not only in chat.
- Split work by module; avoid concurrent edits to the same files. Do not revert
  another contributor's work or rewrite shared branch history without agreement.
- Keep PRs focused and reviewable. Include behavior, tests, remaining limitations,
  and follow-up work. Preserve unrelated working-tree changes.
- Every endpoint method and stop policy must be bounded. Attempt both stop
  policies even when one fails; do not hide partial-send or stop failures.
- State freshness uses local monotonic reception time, not robot timestamps.
  Reception skew is not execution skew. Sequential sends are not atomic.
- Targets are in each robot's base frame unless an explicit calibrated transform
  is applied. Wire quaternions use xyzw; distances are meters.
- Do not silently change gains, select gravity compensation as a universal stop,
  or add blocking work/allocations/network I/O to the RT callback.

## Validation and handoff

- Run `python3 -m pytest tests/test_dual_arm.py` for coordination changes, then
  the full Python suite for shared client/state API changes.
- Build docs with `python3 -m mkdocs build --strict` after documentation changes.
- Use mock endpoints and FakeDaemon for routine tests. Do not start hardware
  services or execute robot motion as part of automated verification.
- Mark real-robot evaluation explicitly as pending until actually performed.
  Passing mocks does not establish synchronization, contact safety, or RT timing.
- Follow-up daemon work: leases/fencing, scheduled buffered execution, local
  watchdogs, clock synchronization, and acknowledged coordination protocol.
