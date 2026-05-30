from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnExecutionComplete
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import launch_ros.descriptions


def generate_launch_description():

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("meldog_description"),
                    "description",
                    "meldog_real.urdf.xacro",
                ]
            )
        ]
    )
    
    # robot_description = [{'robot_description': launch_ros.descriptions.ParameterValue(
    #     Command(
    #     [
    #         PathJoinSubstitution([FindExecutable(name="xacro")]),
    #         " ",
    #         PathJoinSubstitution(
    #             [
    #                 FindPackageShare("meldog_description"),
    #                 "description",
    #                 "meldog_real.urdf.xacro",
    #             ]
    #         )
    #     ]
    #     ), value_type=str)}]
    
    robot_description = {"robot_description": launch_ros.parameter_descriptions.ParameterValue(
        robot_description_content, value_type=str)}
        
    # robot_description = {"robot_description": robot_description_content}
    
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
    )

   
    nodes = [
        robot_state_pub_node,
        rviz_node,
    ]

    return LaunchDescription(nodes)