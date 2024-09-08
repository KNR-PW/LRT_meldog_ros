import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch_ros.parameter_descriptions import ParameterValue

from launch_ros.actions import Node


def generate_launch_description():

   # Declare arguments
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "package",
            default_value="meldog_description",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "urdf_file",
            default_value="meldog_gazebo.urdf.xacro",
        )
    )
     # Initialize Arguments
    package_name = LaunchConfiguration("package")
    urdf_file = LaunchConfiguration("urdf_file")

    # Get URDF via xacro
    robot_description_content = ParameterValue(
        Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare(package_name),
                    "description",
                    urdf_file,
                ]
            ),
        ]
    ), value_type=str)
    robot_description = {"robot_description": robot_description_content}
    use_sim_time = {"use_sim_time": True}

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description, use_sim_time],
    )


    spawn_entity = Node(package='ros_gz_sim', executable='create',
                    arguments=['-topic', 'robot_description',
                                '-name', 'Meldog'],
                    output='screen')






    # Nodes
    nodes = [
        robot_state_pub_node,
        spawn_entity,
    ]
    return LaunchDescription(declared_arguments + nodes)