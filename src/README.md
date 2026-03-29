# ROS 2 Workspace Packages

This workspace contains a perception-to-motion pipeline for a 7-DOF manipulator in simulation.

## Packages

- `msgs_pkg` - Custom ROS 2 interfaces (`msg` and `action`) shared by other packages.
- `realsense_camera_pkg` - Camera simulation/spawn utilities and image topic bridge.
- `segmenta` - Perception nodes for object detection and frame transforms.
- `moveit_7sk` - MoveIt service API for arm and gripper control.
- `mover` - Python client + sequence runner that calls `moveit_7sk` services.

## Typical End-to-End Flow

1. Start simulation and camera feeds (`realsense_camera_pkg`).
2. Start detection (`segmenta`).
3. Start motion API (`moveit_7sk`).
4. Execute a pick/place sequence (`mover`).

## Build

From workspace root (`/home/pavan/p_ws`):

```bash
colcon build --symlink-install
source install/setup.bash
```

## Quick Start

start gazebo and moveit first  (refer `rpd_manpulator_7sk` repo username-: [SriChatur24])


```bash
#Terminal 1: Start gazebo: 
ros2 launch rpd_manipulator_7sk_bringup rpd_manipulator_7sk.launch.py use_sim:=true

#Terminal 2: Start Moveit: 
ros2 launch rpd_manipulator_7sk_moveit_config rpd_manipulator_7sk_moveit.launch.py use_sim:=true

# Terminal 3: Spawn scene objects first
ros2 run realsense_camera_pkg spawn_cubes

# Terminal 4: Start MoveIt service node
ros2 run moveit_7sk moveit_7sk_node

# Terminal 5: Run motion test sequence
ros2 run mover test_sequence
```

For package-specific details, see each package `README.md`.
