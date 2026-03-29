# moveit_7sk

ROS 2 C++ package that exposes a simple service-based API over MoveIt for controlling a 7-DOF arm and gripper.

## What It Provides

- Node: `moveit_7sk_node` (`src/moveit_7sk_node.cpp`)
- Services:
  - `go_to_named_pose` (`moveit_7sk/srv/GoToNamedPose`)
  - `go_to_pose` (`moveit_7sk/srv/GoToPose`)
  - `set_gripper` (`moveit_7sk/srv/SetGripper`)
- Service interface available but not currently advertised by the node:
  - `moveit_7sk/srv/AddNamedPose`

## Build

```bash
cd /home/pavan/p_ws
colcon build --symlink-install --packages-select moveit_7sk
source install/setup.bash
```

## Run

```bash
ros2 run moveit_7sk moveit_7sk_node
```

Optional launch file in this package:

```bash
ros2 launch moveit_7sk moveit_7sk.launch.py
```

## Parameters

Defined in `config/params.yaml`:

- `arm_group` (default: `arm`)
- `gripper_group` (default: `gripper`)
- `velocity_scaling` (default: `1.0`)
- `acceleration_scaling` (default: `1.0`)
- `named_poses_file` (default: `named_poses.yaml`)

## Service Examples

Go to a named pose:

```bash
ros2 service call /go_to_named_pose moveit_7sk/srv/GoToNamedPose "{pose_name: ready}"
```

Go to Cartesian target with RPY:

```bash
ros2 service call /go_to_pose moveit_7sk/srv/GoToPose "{
  x: 0.2,
  y: 0.3,
  z: 0.15,
  roll: -1.57,
  pitch: 0.0,
  yaw: 0.0,
  planning_frame: world
}"
```

Set gripper joint target:

```bash
ros2 service call /set_gripper moveit_7sk/srv/SetGripper "{value: 0.02}"
```

## Notes

- `go_to_named_pose` depends on named targets configured in your MoveIt setup.
- `config/named_poses.yaml` currently exists but is empty.
- This package expects a valid robot MoveIt configuration (planning groups, controllers, kinematics).
