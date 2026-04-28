## Summary

- 

## Area

- [ ] runtime
- [ ] harness
- [ ] benchmark
- [ ] data
- [ ] policy
- [ ] agent
- [ ] docs

## Safety

- [ ] This does not bypass `joint_trajectory_guard_node`.
- [ ] This does not enable real hardware torque by default.
- [ ] This does not describe fake-runtime evidence as real-robot evidence.
- [ ] Protected hardware paths are untouched, or the safety impact is explained below.

## Verification

- [ ] `make ci-local`
- [ ] `make build` if ROS 2 Humble is available
- [ ] `make smoke` if ROS 2 Humble is available

## Evidence

Commands, logs, run cards, bags, screenshots, or artifact links:

