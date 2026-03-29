# realsense_camera_pkg

Camera simulation and scene spawner utilities for ROS 2 + Gazebo Harmonic.

## Nodes

- `ros2 run realsense_camera_pkg spawn_cam`
  - Spawns the RealSense camera model from `models/d435i.sdf`.
  - Key parameters:
    - `x`, `y`, `z`
    - `roll`, `pitch`, `yaw`
    - `model_name`

- `ros2 run realsense_camera_pkg spawn_cubes`
  - Spawns default objects (`red_cube`, `blue_cube`) into the world.

- `ros2 run realsense_camera_pkg start_cam`
  - Bridges Gazebo camera topics using `ros_gz_bridge`.
  - Republishes:
    - `/rgb`
    - `/depth`

- `ros2 run realsense_camera_pkg cam`
  - Optional real-camera publisher using `pyrealsense2`.
  - Publishes `/rgb`, `/depth`, `/intrinsics`.

## Build

```bash
cd /home/pavan/p_ws
colcon build --symlink-install --packages-select realsense_camera_pkg
source install/setup.bash
```

## Typical Simulation Flow

```bash
# Terminal 1: Gazebo
ros2 launch ros_gz_sim gz_sim.launch.py gz_args:="-r empty.sdf"

# Terminal 2: Spawn models
ros2 run realsense_camera_pkg spawn_cam
ros2 run realsense_camera_pkg spawn_cubes

# Terminal 3: Start camera bridge
ros2 run realsense_camera_pkg start_cam
```

## Topics

- Input from Gazebo:
  - `/camera/color/image_raw`
  - `/camera/depth/image_rect_raw`
  - `/camera/color/camera_info`

- Republished for perception:
  - `/rgb`
  - `/depth`

## Dependencies

- `rclpy`
- `ros_gz_interfaces`
- `geometry_msgs`
- `tf_transformations`
- `realsense2_description`
