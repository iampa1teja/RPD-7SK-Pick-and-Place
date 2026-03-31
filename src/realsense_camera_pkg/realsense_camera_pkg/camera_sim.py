import rclpy 
from rclpy.node import Node 
from sensor_msgs.msg import Image 
import subprocess
import time 

class CameraSimNode(Node): 
    def __init__(self): 
        super().__init__("camera_sim") 
        self.bridge_process = subprocess.Popen([
            'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
            '/camera/color/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/depth/image_rect_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/color/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
        ])
        self.get_logger().info("Bridge started, waiting for topics...")
        time.sleep(2.0)  
        
        self.rgb_subscriber_ = self.create_subscription(Image , "/camera/camera/color/image_raw",self.rgb_callback,60) 
        self.depth_subscriber_ = self.create_subscription(Image, "/camera/camera/depth/image_rect_raw",self.depth_callback,60)  

        self.rgb_publisher_ = self.create_publisher(Image, "/rgb",60) #supports upto 60 FPS 
        self.depth_publisher_ = self.create_publisher(Image , "/depth", 60) 

        self.get_logger().info("CAMERA NODE STARTED")

    
    def rgb_callback(self, msg):
        self.rgb_publisher_.publish(msg)

    def depth_callback(self, msg):
        self.depth_publisher_.publish(msg)

def main(args = None): 
    rclpy.init(args = args) 
    node = CameraSimNode()
    rclpy.spin(node)
    rclpy.shutdown() 

if __name__ == "__main__":
    main()