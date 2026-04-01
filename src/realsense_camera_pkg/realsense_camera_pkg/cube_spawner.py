import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
import subprocess
import os

class CubeSpawner(Node):
    def __init__(self):
        super().__init__("cube_spawner")

        pkg_share = get_package_share_directory('realsense_camera_pkg')

        cubes = [
            {
                "name"  : "red_cube",
                "sdf"   : os.path.join(pkg_share, 'models', 'red_cube.sdf'),
                "x"     : "0.1",   
                "y"     : "0.338",
                "z"     : "0.01",   
            },
            {
                "name"  : "blue_cube",
                "sdf"   : os.path.join(pkg_share, 'models', 'blue_cube.sdf'),
                "x"     : "-0.1",  
                "y"     : "0.338",
                "z"     : "0.01",
            },
        ]

        for cube in cubes:
            result = subprocess.run([
                'ros2', 'run', 'ros_gz_sim', 'create',
                '-world', 'empty',
                '-file',  cube['sdf'],
                '-name',  cube['name'],
                '-x',     cube['x'],
                '-y',     cube['y'],
                '-z',     cube['z'],
            ], capture_output=True, text=True)

            if result.returncode == 0:
                self.get_logger().info(f"Spawned {cube['name']}")
            else:
                self.get_logger().error(f"Failed to spawn {cube['name']}: {result.stderr}")


def main(args=None):
    rclpy.init(args=args)
    node = CubeSpawner()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()