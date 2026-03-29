import rclpy 
from rclpy.node import Node 

import pyrealsense2 as rs
import numpy as np 
import cv2 

from sensor_msgs.msg import CameraInfo

from sensor_msgs.msg import Image 
from cv_bridge import CvBridge

class RealSenseNode(Node): 
    def __init__(self):
        super().__init__("realsense_node")

        self.rgb_pub = self.create_publisher(Image, '/rgb',60)
        self.depth_pub = self.create_publisher(Image, '/depth', 60)
        self.camera_info_pub = self.create_publisher(CameraInfo, '/intrinsics', 60)
         
        self.bridge = CvBridge() 

        self.pipeline = rs.pipeline()

        config = rs.config() 
        config.enable_stream(rs.stream.color , 640, 480, rs.format.bgr8, 30)
        config.enable_stream(rs.stream.depth, 640, 480 , rs.format.z16, 30)

        self.pipeline.start(config) 

        self.align = rs.align(rs.stream.color) 

        self.timer = self.create_timer(1/30 , self.publish_frames)

        self.get_logger().info("Realsense ROS2 node started ")

    def publish_frames(self):
        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align.process(frames)
        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()

        if not depth_frame or not color_frame:
            return

        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        rgb_msg = self.bridge.cv2_to_imgmsg(color_image, encoding='bgr8')
        depth_msg = self.bridge.cv2_to_imgmsg(depth_image, encoding='16UC1')

        self.rgb_pub.publish(rgb_msg)
        self.depth_pub.publish(depth_msg)
        camera_info_msg = CameraInfo()
        camera_info_msg.width = 640
        camera_info_msg.height = 480
        camera_info_msg.k = [1.0]*9

        camera_info_msg.header.stamp = self.get_clock().now().to_msg()
        camera_info_msg.header.frame_id = "camera_link"

        self.camera_info_pub.publish(camera_info_msg)
        self.get_logger().info("Intrinsics published")


def main(args=None):
    rclpy.init(args=args)
    node = RealSenseNode()
    rclpy.spin(node)

    node.pipeline.stop()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
