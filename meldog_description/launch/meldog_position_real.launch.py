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
    
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("meldog_description"),
            "controllers",
            "meldog_position_controllers.yaml",
        ]
    )
    
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output="both",
         remappings=[
            ("~/robot_description", "/robot_description"),
        ],
    )
    
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager", "--param-file", robot_controllers],
    )
    
    imu_sensor_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["imu_sensor_broadcaster", "--controller-manager", "/controller_manager", "--param-file", robot_controllers],
    )

    position_controllers_spawner= Node(
        package="controller_manager",
        executable="spawner",
        arguments=["forward_position_controller", "--controller-manager", "/controller_manager", "--param-file", robot_controllers],
    )

    diagnostic_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diagnostic_broadcaster", "--controller-manager", "/controller_manager", "--param-file", robot_controllers],
    )
    
    delay_joint_state_broadcaster_after_robot_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=position_controllers_spawner,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )
    
    delay_imu_sensor_broadcaster_after_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[imu_sensor_broadcaster_spawner],
        )
    )

    delay_diagnostic_broadcaster_after_imu_senor_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=imu_sensor_broadcaster_spawner,
            on_exit=[diagnostic_broadcaster_spawner],
        )
    )

    nodes = [
        control_node,
        #robot_state_pub_node,
        position_controllers_spawner,
        delay_joint_state_broadcaster_after_robot_controller_spawner,
        delay_imu_sensor_broadcaster_after_joint_state_broadcaster,
        delay_diagnostic_broadcaster_after_imu_senor_broadcaster,
    ]

    return LaunchDescription(nodes)