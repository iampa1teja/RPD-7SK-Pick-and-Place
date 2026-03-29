# segmenta

Segmentation and detection package for ROS 2.

## Nodes

- `ros2 run segmenta start`
  - Subscribes: `/rgb`, `/depth`, `/intrinsics`
  - Publishes: `/cam_detections`, `/detections/viz`
  - Parameters:
    - `weights_path` (default: `weights/best_model.pth`)
    - `score_thresh` (default: `0.4`)

- `ros2 run segmenta detect`
  - Subscribes: `/cam_detections`
  - Publishes: `/detections`
  - Applies fixed camera-to-world transform.

## Build

```bash
cd /home/pavan/p_ws
colcon build --symlink-install --packages-select msgs_pkg segmenta
source install/setup.bash
```

## Run

```bash
# 1) Start simulator and camera package in separate terminals
ros2 run realsense_camera_pkg spawn_cam
ros2 run realsense_camera_pkg spawn_cubes
ros2 run realsense_camera_pkg start_cam

# 2) Start segmentation
ros2 run segmenta start

# 3) Optional: convert detections to world frame
ros2 run segmenta detect
```

## Python Dependencies

Install from package requirements:

```bash
pip install -r /home/pavan/p_ws/src/segmenta/req.txt
```

Common runtime libraries include `torch`, `opencv-python`, `cv_bridge`, and `message_filters`.

## Outputs

- `/cam_detections` (`msgs_pkg/DetectionArray`): detections in camera frame.
- `/detections` (`msgs_pkg/DetectionArray`): transformed detections in world frame.
- `/detections/viz` (`sensor_msgs/Image`): annotated debug image.

### Batch Processing (Advanced)
Modify `segmenta/node.py` to accumulate frames and process in batches for higher throughput.

## Troubleshooting

### ImportError: No module named 'torch'

```bash
pip install torch torchvision
# If GPU: install CUDA-compatible versions
# https://pytorch.org/get-started/locally/
```

### Camera topics not found

```bash
# Verify camera node is running
ros2 topic list | grep -E rgb|depth

# If missing, start camera simulation
ros2 run realsense_camera_pkg spawn_cam
ros2 run realsense_camera_pkg start_cam
```

### Detections empty or confidence too low

- Lower `score_thresh` parameter: `-p score_thresh:=0.2`
- Verify model weights are loaded: check console output for warnings
- Ensure objects in scene match training dataset

### TF2 lookup timeout in detect node

```bash
# Verify TF tree
ros2 run tf2_tools view_frames
pdf view_frames.pdf
```
- Ensure `camera_info` is published with correct `frame_id`
- Check `static_transform_publisher` for camera→world transform

### High latency / dropped frames

- Reduce `score_thresh` processing overhead (high threshold=faster)
- Disable visualization: remove `/detections/viz` publication
- Run on GPU for faster inference
- Reduce image resolution

## File Structure

```
segmenta/
  segmenta/
    __init__.py
    node.py                 # Main segmentation node (start)
    transform_sim.py        # TF2 transform node (detect)
    model.py                # Model loading and inference logic
  weights/
    best_model.pth          # Trained model weights
    best.pth                # Alternative weights
  req.txt                   # Python dependencies
  package.xml
  setup.py
  setup.cfg
  README.md
  test/                     # Unit tests (copyright, flake8, pep257)
```

## Related Packages

- **realsense_camera_pkg** — Camera simulation / bridging
- **move_7sk** — Pick-place action servers that consume detections
- **msgs_pkg** — Detection message definitions

## Advanced Topics

### Custom Detection Post-Processing

Edit `segmenta/model.py`:
```python
def postprocess(self, segmentation_mask, depth_image, camera_info):
    """
    Custom logic to extract detections from segmentation.
    
    - Cluster pixels by class
    - Filter by region size
    - Project to 3D
    - Return Detection[] array
    """
    pass
```

### Multi-Class Support

Define class mapping in `segmenta/node.py`:
```python
CLASS_NAMES = {
    0: "background",
    1: "red_cube",
    2: "blue_cube",
    3: "gripper",
    ...
}
```

### Dynamic Threshold Adjustment

Subscribe to parameter changes:
```python
def add_on_set_parameters_callback(self, callback):
    # Adjust score_thresh at runtime
```

## Contributing

- Follow PEP 8 style (`ament_flake8`)
- Add docstrings (`ament_pep257`)
- Test with: `colcon test --packages-select segmenta`
- Document model architecture and training strategy

## References

- PyTorch: https://pytorch.org/
- timm (PyTorch Image Models): https://github.com/rwightman/pytorch-image-models
- ROS 2 Sensor Messages: https://github.com/ros2/common_interfaces
- TF2 Frame Transformations: https://docs.ros.org/en/humble/Concepts/Intermediate/Tf2/Introduction-To-Tf2.html

## License

Apache License 2.0 — See LICENSE file
