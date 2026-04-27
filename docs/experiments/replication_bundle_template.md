# Replication Bundle Template

Use this template for any public benchmark result, thesis result, paper figure, or release claim.

## Result Identity

```text
result id:
claim or benchmark:
repo commit:
repo tag:
date recorded:
operator:
location:
```

## Hardware Manifest

```text
robot:
XLeRobot version:
base variant:
arm configuration:
servo model:
motor board:
camera:
compute:
power supply:
cabling notes:
known hardware deviations:
```

## Software Manifest

```text
operating system:
ROS 2 distro:
workspace build command:
launch command:
environment variables:
external dependencies:
```

## Calibration Bundle

```text
motor id map:
joint zero offsets:
joint signs:
joint limits:
camera intrinsics:
camera extrinsics:
URDF file:
mesh source:
calibration files:
```

## Task Protocol

```text
task:
object:
table or workspace:
lighting:
start pose:
target pose:
reset procedure:
success criterion:
failure criterion:
abort criterion:
exclusions:
```

## Run Card

```text
run id:
repo commit:
robot hardware manifest:
calibration bundle:
task:
object:
start state:
target state:
operator reset:
launch command:
bag path:
success:
failure tag:
manual interventions:
notes:
```

## Required Evidence

- raw rosbag or documented sample bag
- run table
- analysis script or notebook
- task photos or video when visual setup matters
- calibration files
- launch files and exact command history
- known failure modes
- expected tolerance for reproduction

## First Fake-Hardware Bundle

The first fake-hardware evidence bundle should prove that the ROS contract is observable before the physical robot moves.

Record:

- `/joint_states`
- `/tf`
- `/tf_static`
- `/harness/events`
- `/left_arm_controller/joint_trajectory`
- `/left_arm_controller/guarded_joint_trajectory`
- `/right_arm_controller/joint_trajectory`
- `/right_arm_controller/guarded_joint_trajectory`

Command target:

```bash
ros2 launch xle_bringup fake_one_arm.launch.py
```

Bag target:

```bash
ros2 bag record /joint_states /tf /tf_static /harness/events /left_arm_controller/joint_trajectory /left_arm_controller/guarded_joint_trajectory /right_arm_controller/joint_trajectory /right_arm_controller/guarded_joint_trajectory
```
