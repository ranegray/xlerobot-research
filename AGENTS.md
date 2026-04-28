# Agent Contribution Rules

This repository is hardware-adjacent research infrastructure. Coding agents may help freely on low-risk surfaces, but must not weaken the hardware path or overstate fake-runtime evidence.

## Default Safe Areas

Agents may make PR-sized changes to:

- docs, diagrams, run cards, and replication-bundle templates
- tests and test fixtures
- CI, lint, formatting, and package inventory checks
- fake-hardware nodes and fake-runtime smoke scripts
- fake trajectory fixtures under `tests/fixtures/trajectories/`
- schemas, analysis scripts, and benchmark scaffolds
- launch documentation and read-only launch inspection tools

## Protected Areas

Agents must be conservative in:

- `ros2_ws/src/xle_hardware/xle_hardware/bus1_sts3215_node.py`
- calibration, servo-ID assignment, and motor-direction code
- scripts that can move real motors
- launch files or parameters that can enable torque
- guard, harness, abort, and limit semantics
- URDF joint limits, frame names, and hardware manifests

Touch protected areas only when the user explicitly asks or when a failing test proves the need. Explain the safety impact in the PR or final note.

## Hard Rules

- Do not bypass `joint_trajectory_guard_node` for generated or agent-authored commands.
- Do not change torque defaults from safe/read-only to motion-enabled.
- Do not claim real robot evidence from fake hardware, RViz, or static checks.
- Do not introduce MoveIt, `ros2_control`, Nav2, or broad autonomy as core dependencies without explicit approval.
- Keep the base parked and the v0.2/v0.3 path left-arm-first unless the roadmap changes.

## Required Checks

Before handing off a normal PR-sized change, run:

```bash
make lint
make test
```

When ROS 2 Humble is available, also run:

```bash
make build
make smoke
```

If ROS 2 is not available locally, say that clearly and include which local checks passed.
