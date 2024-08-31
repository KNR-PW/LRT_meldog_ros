from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Start RViz2 automatically with this launch file.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "package",
            default_value="meldog_description",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "urdf_file",
            default_value="meldog_core.urdf.xacro",
        )
    )


    # Initialize Arguments
    package_name = LaunchConfiguration("package")
    urdf_file = LaunchConfiguration("urdf_file")

    # Get URDF via xacro
    robot_description_content = Command(
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
    )
    robot_description = {"robot_description": robot_description_content}

    # rviz_config_file = PathJoinSubstitution(
    #     [FindPackageShare("ros2_control_demo_description"), "rrbot/rviz", "rrbot.rviz"]
    # )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    joint_state_publisher_gui = Node(
        package = "joint_state_publisher_gui",
        executable = "joint_state_publisher_gui",
    )

    # Initialize Arguments
    gui = LaunchConfiguration("gui")

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        # arguments=["-d", rviz_config_file],
        condition=IfCondition(gui),
    )



    nodes = [
        robot_state_pub_node,
        joint_state_publisher_gui, 
        rviz_node 
    ]

    return LaunchDescription(declared_arguments + nodes)