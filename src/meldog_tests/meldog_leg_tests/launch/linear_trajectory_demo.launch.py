from launch import LaunchDescription
from launch_ros.actions import Node



LENGTH_1 = 0.225
LENGTH_2 = 0.225
START_POSITION = [0.0, -0.30]
RADIUS = 0.1
ANGULAR_VALOCITY = 3.14
def generate_launch_description():
    inverse_kinematics = Node(
        package = 'meldog_leg_tests',
        executable = 'inverse_kinematics_2D',
        parameters = [{
            'length_1': LENGTH_1,
            'length_2': LENGTH_2,
            'start_position': START_POSITION
        }],
        output = 'screen'
    )
    forward_kinematics = Node(
        package = 'meldog_leg_tests',
        executable = 'forward_kinematics_2D',
        parameters = [{
            'length_1': LENGTH_1,
            'length_2': LENGTH_2
        }],
        output = 'screen'
    )
    linear_trajectory = Node(
        package = 'meldog_leg_tests',
        executable = 'linear_trajectory',
        parameters = [{
            'start_position': START_POSITION,
            'radius': RADIUS,
            'angular_velocity': ANGULAR_VALOCITY
        }],
        output = 'screen'
    )

    nodes = [inverse_kinematics, linear_trajectory]
    ld = LaunchDescription(nodes)
    return ld
