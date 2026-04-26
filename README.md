# xlerobot-research

A ROS 2 research stack for reproducible XLeRobot experiments.

## Status

Status: early bring-up.

This repository is under active development. The initial supported target is the XLeRobot 0.4.0 two-wheel servo variant. The first stable milestone is fake hardware plus one-arm ROS 2 bring-up, not full autonomy.

## What This Is

`xlerobot-research` provides a ROS 2 stack for XLeRobot research: robot description, custom hardware nodes, fake hardware, RealSense integration, safety gates, episode logging, benchmark protocols, and reproducibility bundles.

The stack is built around a few working principles:

- custom ROS 2 nodes first
- standard ROS message types and naming conventions where possible
- fake hardware and real hardware share the same command contract
- every result should be reproducible by another researcher with matching hardware
- agent interfaces come after the robot is observable and safe

## Supported Hardware

Initial support:

- XLeRobot 0.4.0 two-wheel servo variant
- SO101 arm configuration used by Pincer
- STS3215 servo bus
- Intel RealSense D435if
- Jetson-class onboard compute
- ROS 2 Humble

Each release should name the exact tested hardware manifest.

## Quick Start

The first quick start will be simulation/fake-hardware first:

```bash
git clone https://github.com/ranegray/xlerobot-research.git
cd xlerobot-research/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
ros2 launch xle_bringup fake_one_arm.launch.py
```

This command is the first intended milestone. It should eventually bring up:

- `robot_state_publisher`
- fake joint states
- `/tf`
- guarded trajectory forwarding
- one arm-controller-style command surface
- harness event stream

See `docs/simulation/fake_hardware.md` for the first fake command and bag target.

## Architecture

Planned package roles:

| package | role |
| --- | --- |
| `xle_description` | URDF, meshes, frame names, robot-state launch path |
| `xle_bringup` | launch files and configuration composition |
| `xle_hardware` | STS3215 bus access, motor discovery, calibration loading, joint states, guarded commands |
| `xle_fake_hardware` | interface-compatible fake motors for testing without the real robot |
| `xle_perception` | initial RealSense-backed detector and object-pose output |
| `xle_harness` | episodes, reset state, abort gates, verifier events, evidence logging |
| `xle_benchmarks` | benchmark task definitions and repeated-run protocols |
| `xle_agent_interfaces` | boundary between coding agents and the robot API |

## Core ROS 2 Interface

The initial public contract follows ROS controller conventions where possible.

Global robot state:

- `/robot_description`
- `/joint_states`
- `/tf`
- `/tf_static`

Arm controllers:

- `/left_arm_controller/joint_trajectory`
- `/left_arm_controller/guarded_joint_trajectory`
- `/right_arm_controller/joint_trajectory`
- `/right_arm_controller/guarded_joint_trajectory`
- `/left_arm_controller/controller_state`
- `/right_arm_controller/controller_state`
- `/left_arm_controller/follow_joint_trajectory`
- `/right_arm_controller/follow_joint_trajectory`

Gripper controllers:

- `/left_gripper_controller/gripper_cmd`
- `/right_gripper_controller/gripper_cmd`

Perception and harness:

- `/camera/color/image_raw`
- `/camera/depth/image_rect_raw`
- `/harness/events`
- `/episode/state`
- `/episode/abort`

Standard ROS message types should be used before custom messages.

## Simulation Ladder

Simulation enters as a contract test before it becomes a physics claim.

1. Fake hardware for interface and harness testing.
2. Kinematic validation with URDF, TF, and RViz.
3. Physics simulation only when it answers a concrete research question.

## Hardware Bring-Up

Initial bring-up should cover:

- motor ID discovery
- bus topology
- calibration
- joint signs
- measured joint limits
- safe one-joint command
- stow pose
- first real rosbag

## Reproducibility

Every published benchmark, paper result, or thesis result should point to a replication bundle:

- repo tag
- hardware manifest
- calibration files
- launch commands
- task protocol
- reset protocol
- rosbags
- run table
- analysis script
- expected tolerance
- known exclusions

Use `docs/experiments/replication_bundle_template.md` as the starting template.

## Safety Model

Generated or agent-written code must never bypass the harness or command guard.

Initial safety rules:

- commands are bounded before they reach hardware
- collision or protocol violations abort the episode
- manual interventions are logged
- agent access is mediated through explicit interfaces

## Roadmap

- v0.1 visible robot model
- v0.2 fake hardware path
- v0.3 one-arm hardware read path
- v0.4 one-arm guarded motion
- v0.5 RealSense and perception bridge
- v0.6 first camera-guided reach or grasp
- v0.7 episode harness
- v0.8 benchmark task suite
- v0.9 reproducibility bundle
- v0.10 research documentation
- v1.0 public research release

## Non-Goals

For the initial release:

- no MoveIt Pro dependency
- no required `ros2_control`
- no Nav2 or base autonomy
- no bimanual manipulation claim
- no generalized household manipulation claim
- no benchmark claims before real target-config motion works

## Relationship To XLeRobot

This is not the upstream XLeRobot repository. It is a research stack built for XLeRobot hardware.

The initial robot model is vendored from [Vector-Wangel/XLeRobot](https://github.com/Vector-Wangel/XLeRobot) at commit `137865981ca9e828d0923804cf77ededd22c7816`. The default ROS model is a parked-base derivative for early fake-hardware bring-up; the vendor copy is preserved next to it. See `third_party/XLeRobot-URDF-PROVENANCE.md` and `third_party/XLeRobot-LICENSE` for source and license details.

Credit and license handling should remain explicit for any imported driver, mesh, calibration source, image, or external asset.

## License

License is not selected yet. Prefer a permissive license for code and clear separate treatment for imported meshes, CAD, images, and external assets.
