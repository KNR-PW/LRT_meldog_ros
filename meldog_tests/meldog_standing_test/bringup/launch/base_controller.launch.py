import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    config = os.path.join(
        get_package_share_directory('meldog_standing_test'),
        'bringup', 'config', 'base_controller_config.yaml'
        )
        
    node=Node(
        package = 'meldog_standing_test',
        name = 'base_controller',
        executable = 'base_controller',
        parameters = [config]
    )
    ld.add_action(node)
    return ld