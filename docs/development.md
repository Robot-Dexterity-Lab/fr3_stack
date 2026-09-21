# Development entry points

Start with the [agent project guide](https://github.com/Robot-Dexterity-Lab/fr3_stack/blob/main/AGENTS.md)
for the workstation/NUC architecture, command and feedback paths, task-specific
code entry points, and verification commands. It helps a new agent understand
the repository before making changes.

## Boundaries

| Module | Responsibility |
| --- | --- |
| `dual_arm.py` | Transport-independent paired validation, control loop, state machine, fault propagation |
| `dual_arm_robot.py` | Adapter from the endpoint protocol to the public Robot API |
| `robot.py`, `state.py` | Independent single-arm transport and local receive timestamps |
| `proto/fr3.capnp`, `src/main.cpp` | Wire protocol and daemon scheduling |
| `src/controllers/` | Low-level controller math and real-time behavior |

Agree on shared interfaces and failure semantics before splitting work. Keep
changes focused by module and preserve unrelated working-tree edits. Each PR
should explain behavior, tests, remaining limitations, and handoff tasks.

## Local verification

```bash
python3 -m pip install -e '.[dev]'
python3 -m pytest tests/test_dual_arm.py
python3 -m pytest
python3 -m pip install -r requirements-docs.txt
python3 -m mkdocs build --strict
```

Tests use mocks and local FakeDaemons; they are not real-robot evaluation.
Do not run hardware motion as an ordinary test step.

## Documentation publishing

`docs/` is the source of the GitHub Pages website. `mkdocs.yml` defines navigation
and presentation; `site/` is generated output and must not be maintained by hand.
The Deploy documentation workflow builds strictly, uploads a Pages artifact,
and deploys it with GitHub's Pages action.

The initial preview deploys from `feat/dual-arm-coordinator`. After merge,
documentation changes on `main` also deploy automatically. This is one website;
a deployment replaces its previous contents. The feature-branch trigger can be
removed when the preview branch is retired.

Real evaluation remains pending. Record measured results in the docs only after
they have been obtained, with hardware versions and test conditions.
