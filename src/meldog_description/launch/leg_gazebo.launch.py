from launch import LaunchDescription
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import get_package_share_path

def generate_launch_description():

    urdf_path = os.path.join(get_package_share_path('meldog_description'),
                             'urdf','standalone_leg.urdf.xacro')

    robot_description = ParameterValue(Command(['xacro ',urdf_path]),
                                       value_type=str)
    
    rviz_config_path = os.path.join(get_package_share_path('meldog_description'),
                                    'rviz','leg_config.rviz')

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{'robot_description' : robot_description,
                     "use_sim_time" : True}]
    )
    
    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui"
    )
    
    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=[
            '-d', rviz_config_path
        ]
    )
    
    ros_distro = os.environ["ROS_DISTRO"]
    physics_engine="" if ros_distro=="humble" else "--physics-engine gz-physics-bullet-featherstone-plugin"
    
    gazebo_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[str(Path(get_package_share_directory('meldog_description')).parent.resolve())]
    )
    
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch'), '/gz_sim.launch.py']),
            launch_arguments={'gz_args': ['-r -v -v4 empty.sdf'], 'on_exit_shutdown': 'true'}.items()
        )
    
    spawn_entity = Node(package='ros_gz_sim', executable='create',
                    arguments=['-topic', 'robot_description',
                                '-name', 'Meldog'],
                    output='screen')
    
    # gz_ros2_bridge = Node(
    #     package='ros_gz_bridge',
    #     executable='parameter_bridge',
    #     arguments=[
    #         "clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"
    #     ]
    # )

    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz2_node,
        gazebo_resource_path,
        gazebo,
        spawn_entity,
        # gz_ros2_bridge,
        
    ])