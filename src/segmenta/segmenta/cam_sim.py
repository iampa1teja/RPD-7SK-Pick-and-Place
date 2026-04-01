import rclpy 
from rclpy.node import Node
from sensor_msgs.msg import Image , CameraInfo 
from msgs_pkg.msg import DetectionArray , Detection
import message_filters
from pathlib import Path
import torch 
import numpy as np 
from cv_bridge import CvBridge 
from ament_index_python.packages import get_package_share_directory
from .model import Segmenta 
import cv2
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

DEVICE = "cuda" if torch.cuda.is_available() else "cpu"

print(f"Using device {DEVICE}") 

CFG = {
    "backbone"      :   "resnet50", 
    "num_classes"   :   5,
    "fpn_ch"        :   256, 
    "fpn_layers"    :   3, 
    "img_h"         :   480, 
    "img_w"         :   680,
    "score_thresh"  :   0.4,
    "nms_iou_thresh":   0.8, 
    "max_dets"      :   10, 
}

CLASS_NAMES = { 
    0 : 'background', 
    1 : 'red_cube', 
    2 : 'blue_cube', 
    3 : 'red_box',
    4 : 'blue_box',
}

CLASS_BGR = {
    1 : (0 , 0 , 255), 
    2 : (255, 0 , 0), 
    3 : (71, 99, 255), 
    4 : (255, 191 , 0),
}


def resolve_weights_path(weights_path: str) -> str:
    path_obj = Path(weights_path).expanduser()
    if path_obj.is_absolute() and path_obj.exists():
        return str(path_obj)

    candidates = [
        Path.cwd() / path_obj,
        Path(__file__).resolve().parents[1] / path_obj,
    ]

    try:
        share_dir = Path(get_package_share_directory("segmenta"))
        candidates.extend([
            share_dir / path_obj,
            share_dir / "weights" / path_obj.name,
        ])
    except Exception:
        pass

    for candidate in candidates:
        if candidate.exists():
            return str(candidate)

    searched = ", ".join(str(candidate) for candidate in candidates)
    raise FileNotFoundError(
        f"Could not find weights file '{weights_path}'. Searched: {searched}"
    )

class SegmentaCamSim(Node):
    def __init__(self): 
        super().__init__("detection_cam_sim_node") 
      
        self.declare_parameter("weights_path", "weights/best_model.pth")
        self.declare_parameter("score_thresh", 0.4)
        self.declare_parameter("cam_x", 0.0)
        self.declare_parameter("cam_y", 0.0)
        self.declare_parameter("cam_z", 0.0)
        self.declare_parameter("cam_roll", 0.0)
        self.declare_parameter("cam_pitch", 0.0)
        self.declare_parameter("cam_yaw", 0.0)
        
        weights_path = self.get_parameter("weights_path").get_parameter_value().string_value
        weights_path = resolve_weights_path(weights_path)
        score_thresh = self.get_parameter("score_thresh").get_parameter_value().double_value
        
        self.cam_x = self.get_parameter("cam_x").get_parameter_value().double_value
        self.cam_y = self.get_parameter("cam_y").get_parameter_value().double_value
        self.cam_z = self.get_parameter("cam_z").get_parameter_value().double_value
        self.cam_roll = self.get_parameter("cam_roll").get_parameter_value().double_value
        self.cam_pitch = self.get_parameter("cam_pitch").get_parameter_value().double_value
        self.cam_yaw = self.get_parameter("cam_yaw").get_parameter_value().double_value
        
        CFG["score_thresh"] = score_thresh

        self.segmenta_model = Segmenta(CFG).to(DEVICE)
        checkpoint = torch.load(weights_path, map_location=DEVICE, weights_only=True)
        self.segmenta_model.load_state_dict(checkpoint["model_state_dict"])
        self.segmenta_model.eval() 
        self.get_logger().info(f"Model loaded on {DEVICE}")

        self.fx = self.fy = self.cy = self.cx = None 
        self.create_subscription(
            CameraInfo, 
            "/camera/camera/color/camera_info", 
            self.camera_info_callback,
            60
        ) 

        self.bridge = CvBridge() 
        
        rgb_sub = message_filters.Subscriber(self, Image, "/camera/camera/color/image_raw") 
        depth_sub = message_filters.Subscriber(self, Image, "/camera/camera/aligned_depth_to_color/image_raw")
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [rgb_sub, depth_sub], queue_size=10, slop = 0.05
        )
        self.sync.registerCallback(self.sync_callback)

        self.det_publisher_ = self.create_publisher(DetectionArray, "/cam_detections_sim", 10)
        self.viz_publisher_ = self.create_publisher(Image, "/detections/viz_sim", 10)
        
        # Transform broadcaster for camera pose
        self.tf_broadcaster = TransformBroadcaster(self)

        self.get_logger().info("Segmentation camera sim node ready")
        
    def optical_to_camera(self, x, y, z):
        return (
            z,     # forward
            -x,    # left
            -y     # up
        )
    
    def apply_camera_transform(self, cam_x_opt, cam_y_opt, cam_z_opt):
        """Apply coordinate transformation: x = cam_y, y = cam_z, z = cam_x"""
        x = cam_y_opt
        y = cam_z_opt
        z = cam_x_opt
        return x, y, z

    def camera_info_callback(self, msg):
        self.fx = msg.k[0]
        self.fy = msg.k[4] 
        self.cx = msg.k[2]
        self.cy = msg.k[5]
    
    def depth_to_xyz(self, u , v , depth_m):
        x = (u - self.cx) * depth_m / self.fx
        y = (v - self.cy) * depth_m / self.fy 
        z = depth_m 
        return x , y , z 

    def _depth_image_to_meters(self, depth_cv, encoding: str):
        # RealSense depth is commonly uint16 in millimeters; float encodings are meters.
        depth_m = depth_cv.astype(np.float32)
        if encoding == "16UC1":
            depth_m = depth_m / 1000.0
        return depth_m
    
    def publish_camera_transform(self, timestamp):
        """Publish camera transform with fixed position and rotation"""
        t = TransformStamped()
        t.header.stamp = timestamp
        t.header.frame_id = "world"
        t.child_frame_id = "camera_link"
        
        # Camera position (editable via parameters)
        t.transform.translation.x = self.cam_x
        t.transform.translation.y = self.cam_y
        t.transform.translation.z = self.cam_z
        
        # Camera rotation from roll, pitch, yaw (editable via parameters)
        # Convert RPY to quaternion
        from tf_transformations import quaternion_from_euler
        qx, qy, qz, qw = quaternion_from_euler(self.cam_roll, self.cam_pitch, self.cam_yaw)
        
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        
        self.tf_broadcaster.sendTransform(t)
    
    def sync_callback(self, rgb_msg, depth_msg):
        if self.fx is None:
            self.get_logger().warn("Waiting for camera intrinsics...")
            return 

        # Publish camera transform
        self.publish_camera_transform(rgb_msg.header.stamp)

        rgb_cv = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="rgb8")
        depth_cv= self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding= "passthrough")
        depth_m = self._depth_image_to_meters(depth_cv, depth_msg.encoding)

        overlay = cv2.cvtColor(rgb_cv, cv2.COLOR_RGB2BGR)
        orig_h , orig_w = overlay.shape[:2] 

        arr    = rgb_cv.astype(np.float32) / 255.0
        tensor = torch.from_numpy(arr).permute(2, 0, 1).float()
        mean   = torch.tensor([0.485, 0.456, 0.406]).view(3, 1, 1)
        std    = torch.tensor([0.229, 0.224, 0.225]).view(3, 1, 1)
        tensor = (tensor - mean) / std

        # ── Inference ─────────────────────────────────────────────────────
        with torch.no_grad():
            instances = self.segmenta_model.predict(tensor.unsqueeze(0).to(DEVICE))[0]

        det_array             = DetectionArray()
        det_array.header      = rgb_msg.header
        det_array.header.frame_id = "camera_link"

        for inst in instances:
            cls_id = inst["class"]
            mask   = inst["mask"].cpu().numpy().astype(np.uint8)  
            color_bgr = CLASS_BGR.get(cls_id, (255, 255, 255))

            colored_mask = np.zeros_like(overlay)
            colored_mask[mask > 0] = color_bgr
            overlay = cv2.addWeighted(overlay, 1.0, colored_mask, 0.75, 0)

            cx_n, cy_n, w_n, h_n = inst["cx"], inst["cy"], inst["w"], inst["h"]
            x1 = int((cx_n - w_n / 2) * orig_w)
            y1 = int((cy_n - h_n / 2) * orig_h)
            x2 = int((cx_n + w_n / 2) * orig_w)
            y2 = int((cy_n + h_n / 2) * orig_h)
            x1 = max(0, min(x1, orig_w - 1))
            y1 = max(0, min(y1, orig_h - 1))
            x2 = max(0, min(x2, orig_w - 1))
            y2 = max(0, min(y2, orig_h - 1))
            if x2 <= x1 or y2 <= y1:
                continue
            cv2.rectangle(overlay, (x1, y1), (x2, y2), color_bgr, 2)
            cv2.putText(
                overlay,
                f"{CLASS_NAMES.get(cls_id, '?')} {inst['obj_score'] * inst['cls_score']:.2f}",
                (x1, y1 - 5),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, color_bgr, 1
            )

            depth_region = depth_m[mask > 0]
            depth_region = depth_region[np.isfinite(depth_region)]
            depth_region = depth_region[depth_region > 0]
            if len(depth_region) == 0:
                continue
            median_depth = float(np.median(depth_region))

            ys, xs = np.where(mask > 0)
            u = float(np.median(xs))
            v = float(np.median(ys))
            
            x_opt, y_opt, z_opt = self.depth_to_xyz(u, v, median_depth)
            x_cam, y_cam, z_cam = self.optical_to_camera(x_opt, y_opt, z_opt)
            
            # Apply coordinate transformation: x = cam_y, y = cam_z, z = cam_x
            x, y, z = self.apply_camera_transform(x_cam, y_cam, z_cam)

            det            = Detection()
            det.class_name = CLASS_NAMES.get(cls_id, "unknown")
            det.class_id   = int(cls_id)
            det.confidence = float(inst["obj_score"] * inst["cls_score"])
            det.x          = float(x)
            det.y          = float(z)
            det.z          = float(y)
            det_array.detections.append(det)

        self.det_publisher_.publish(det_array)

        viz_msg = self.bridge.cv2_to_imgmsg(overlay, encoding="bgr8")
        viz_msg.header = rgb_msg.header
        self.viz_publisher_.publish(viz_msg)

        self.get_logger().debug(f"Published {len(det_array.detections)} detections")


def main(args=None):
    rclpy.init(args=args)
    node = SegmentaCamSim()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
        