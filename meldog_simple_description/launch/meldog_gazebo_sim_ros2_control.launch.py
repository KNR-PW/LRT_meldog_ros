# Copyright 2020 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnProcessStart
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
            default_value="meldog_simple_description",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "urdf_file",
            default_value="meldog_gazebo.urdf.xacro",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "controller_type_file",
            default_value="joint_trajectory_controller_test.yaml",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "controller_type",
            default_value="joint_trajectory_controller",
        )
    )

    # Initialize Arguments
    package_name = LaunchConfiguration("package")
    urdf_file = LaunchConfiguration("urdf_file")
    controller_file = LaunchConfiguration("controller_type_file")
    controller_type = LaunchConfiguration("controller_type")


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

    # Set up dictionary parameters:
    robot_description = {"robot_description": robot_description_content}
    use_sim_time = {"use_sim_time": True}

    # Load world for gazebo sim
    world = PathJoinSubstitution(
        [
            FindPackageShare(package_name),
            "worlds",
            'empty.world'
        ]
    )

    # robot_state_publisher node:
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description, use_sim_time],
    )

    # Spawn Meldog:
    spawn_entity = Node(package='ros_gz_sim', executable='create',
                    arguments=['-topic', 'robot_description',
                                '-name', 'Meldog'],
                    output='screen')

    # Load joint_state_broadcaster
    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    # Load joint_trajectory_controller 
    load_joint_effort_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', controller_type],
        output='screen'
    )

    # Bridge between ros2 and Ignition Gazebo
    # ros2_gazebo_sim_bridge =  IncludeLaunchDescription(
    #         PythonLaunchDescriptionSource(
    #             [os.path.join(get_package_share_directory('ros_ign_gazebo'),
    #                           'launch', 'ign_gazebo.launch.py')]),
    #         launch_arguments=[('gz_args', [' -r -v 4 empty.sdf'])])
    gazebo_sim_bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch'), '/gz_sim.launch.py']),
            launch_arguments={'gz_args': ['-r -v -v4 ', world], 'on_exit_shutdown': 'true'}.items()
        )
    
    nodes = [
        gazebo_sim_bridge,

        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[load_joint_state_broadcaster],
            )
        ),

        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action= load_joint_state_broadcaster,
                on_exit=[load_joint_effort_controller],
            )
        ),

        robot_state_pub_node,
        spawn_entity,
    ]
    return LaunchDescription(declared_arguments + nodes)