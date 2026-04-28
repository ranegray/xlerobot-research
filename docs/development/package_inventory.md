# Package Inventory

The ROS 2 workspace currently contains:

| package | build type | role |
| --- | --- | --- |
| `xle_description` | `ament_cmake` | URDF, meshes, frame names, robot-state launch path |
| `xle_bringup` | `ament_cmake` | launch files and configuration composition |
| `xle_hardware` | `ament_python` | STS3215 access, calibration loading, joint states, guarded commands |
| `xle_fake_hardware` | `ament_python` | interface-compatible fake motors for tests and smoke runs |
| `xle_perception` | `ament_python` | initial RealSense-backed detector and object-pose output |
| `xle_harness` | `ament_python` | episodes, reset state, abort gates, verifier events, evidence logging |
| `xle_benchmarks` | `ament_python` | benchmark task definitions and repeated-run protocols |
| `xle_agent_interfaces` | `ament_python` | mediated boundary between coding agents and robot APIs |

Regenerate the checked inventory:

```bash
python3 scripts/check_package_inventory.py
```

