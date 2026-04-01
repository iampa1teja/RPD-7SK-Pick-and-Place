import rclpy 
from rclpy.node import Node 
from ament_index_python.packages import get_package_share_directory
from tf_transformations import quaternion_from_euler
import os
import subprocess



class SpawnNode(Node): 
    def __init__(self): 
        super().__init__("camera_spawn") 
        self.declare_parameter('x', 0.60)  
        self.declare_parameter('y', 0.00) 
        self.declare_parameter('z', 0.10)

        x =     self.get_parameter('x').get_parameter_value().double_value
        y =     self.get_parameter('y').get_parameter_value().double_value
        z =     self.get_parameter('z').get_parameter_value().double_value
        
        self.declare_parameter('roll', 3.14159) # 150 degrees 
        self.declare_parameter('pitch', 3.14159) # assuming 0 
        self.declare_parameter('yaw', 0.0) # assuming 0

        roll =  self.get_parameter('roll').get_parameter_value().double_value
        pitch = self.get_parameter('pitch').get_parameter_value().double_value
        yaw =   self.get_parameter('yaw').get_parameter_value().double_value

        self.declare_parameter("model_name", "realsense_camera")

        model_name = self.get_parameter('model_name').get_parameter_value().string_value

        self.sdf_content = "" 
        pkg_share = get_package_share_directory('realsense_camera_pkg')
        self.sdf_path = os.path.join(pkg_share, 'models', 'd435i.sdf')

        try: 
            with open(self.sdf_path, "r") as file:
                self.sdf_content = file.read()
                self.get_logger().info("Succesfully read the sdf file ")
        except Exception as e: 
            self.get_logger().error(f"Failed to read SDF file exited with this excepthion : {e}")
            return

        result = subprocess.run([
            'ros2', 'run', 'ros_gz_sim', 'create',
            '-world', 'empty',
            '-file', self.sdf_path,
            '-name', model_name,
            '-x', str(x),
            '-y', str(y),
            '-z', str(z),
            '-R', str(roll),
            '-P', str(pitch),
            '-Y', str(yaw),
        ], capture_output=True, text=True)

        if result.returncode == 0:
            self.get_logger().info(f"Successfully spawned {model_name}")
        else:
            self.get_logger().error(f"Spawn failed: {result.stderr}")



def main(args = None): 
    rclpy.init(args=args)
    node = SpawnNode() 
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__": 
    main()