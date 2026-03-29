from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import yaml

def generate_launch_description():

    kinematics_yaml = os.path.join(
        '/home/pavan/c_ws/install/rpd_manipulator_7sk_moveit_config',
        'share/rpd_manipulator_7sk_moveit_config/config/kinematics.yaml'
    )

    with open(kinematics_yaml, 'r') as f:
        kinematics_params = yaml.safe_load(f)

    return LaunchDescription([
        Node(
            package='moveit_7sk',
            executable='moveit_7sk_node',
            name='moveit_7sk_node',
            parameters=[
                kinematics_params,
                {'use_sim_time': True}
            ],
            output='screen'
        )
    ])