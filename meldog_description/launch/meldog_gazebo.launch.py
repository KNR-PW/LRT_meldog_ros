import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory, get_package_share_path
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument,
                            IncludeLaunchDescription, RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

# Loaded, configured and activated one by one in this order. joint_forward_trajectory_controller
# is chained to joint_controller (uses its reference interfaces), so it has to come after it.
CONTROLLERS = [
    'joint_state_broadcaster',
    'joint_controller',
    'imu_sensor_broadcaster',
    'contact_sensors_broadcaster',
    'joint_forward_trajectory_controller',
]


def generate_launch_description():

    urdf_path = os.path.join(get_package_share_path('meldog_description'),
                             'description', 'meldog.urdf.xacro')

    collision_arg = DeclareLaunchArgument(
        'collision', default_value='simple',
        description='Collision model: simple (spheres on the legs, box on the trunk) or mesh (STL)')

    robot_description = ParameterValue(
        Command(['xacro ', urdf_path, ' sim:=gazebo collision:=', LaunchConfiguration('collision')]),
        value_type=str)

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description,
                     'use_sim_time': True}]
    )

    # Lets Gazebo resolve package://meldog_description/... (appended, keeps the user's paths)
    gazebo_resource_path = AppendEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=str(Path(get_package_share_directory('meldog_description')).parent.resolve())
    )

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        ],
        output='screen'
    )

    # Load world for gazebo sim
    world = PathJoinSubstitution(
        [
            FindPackageShare('meldog_description'),
            'worlds',
            'simple.sdf'
        ]
    )

    # -v 2: errors and warnings (-v 4 debug output slows the simulation down)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch'), '/gz_sim.launch.py']),
        launch_arguments={'gz_args': ['-r -v 2 ', world], 'on_exit_shutdown': 'true'}.items()
    )

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'Meldog',
            '-x', '0',
            '-y', '0',
            '-z', '0.55',
        ],
        output='screen'
    )

    controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=CONTROLLERS + ['--controller-manager', '/controller_manager',
                                 '--controller-manager-timeout', '30'],
        output='screen'
    )

    return LaunchDescription([
        collision_arg,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[controller_spawner],
            )
        ),
        robot_state_publisher_node,
        gazebo_resource_path,
        gazebo,
        spawn_entity,
        clock_bridge,
    ])
