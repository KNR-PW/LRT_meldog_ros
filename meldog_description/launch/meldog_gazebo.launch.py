from launch import LaunchDescription
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import get_package_share_path

from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    urdf_path = os.path.join(get_package_share_path('meldog_description'),
                             'description','meldog_core.urdf.xacro')

    robot_description = ParameterValue(Command(['xacro ',urdf_path]),
                                       value_type=str)

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{'robot_description' : robot_description,
                     "use_sim_time" : True}]
    )
    
    ros_distro = os.environ["ROS_DISTRO"]

    # Not used
    physics_engine="" if ros_distro=="humble" else "--physics-engine gz-physics-bullet-featherstone-plugin"
    
    gazebo_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[str(Path(get_package_share_directory('meldog_description')).parent.resolve())]
    )

     # Load world for gazebo sim
    world = PathJoinSubstitution(
        [
            FindPackageShare('meldog_description'),
            "worlds",
            'simple.sdf'
        ]
    )
    
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch'), '/gz_sim.launch.py']),
            launch_arguments={'gz_args': ['-r -v -v4 ', world], 'on_exit_shutdown': 'true'}.items()
        )
    
    spawn_entity = Node(package='ros_gz_sim', executable='create',
                    arguments=['-topic', 'robot_description',
                                '-name', 'Meldog'],
                    output='screen')
    
    
    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    load_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_controller'],
        output='screen'
    )   
    load_trajectory_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_controller'],
        output='screen'
    )
    load_imu_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'imu_sensor_broadcaster'],
        output='screen'
    )



    return LaunchDescription([
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[load_joint_state_broadcaster],
            )
        ),

        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action= load_joint_state_broadcaster,
                on_exit=[load_controller],
            )
        ),

        RegisterEventHandler(
            OnProcessExit(
                target_action=load_controller,
                on_exit=[load_imu_broadcaster],
            )
        ),
        robot_state_publisher_node,
        gazebo_resource_path,
        gazebo,
        spawn_entity,
    ])