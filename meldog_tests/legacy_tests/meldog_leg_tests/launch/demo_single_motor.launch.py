from launch import LaunchDescription
from launch_ros.actions import Node

PI = 3.14

ANGULAR_VELOCITY = PI               # Angular velocity of motion [rad/s]
AMPLITUDE = 2*PI                    # Amplitude of motion [rad]
CHOSEN_MOTOR = 1                    # Motor whose data will be ploted (STARTING FROM 1, NOT 0!)
def generate_launch_description():
    ploter_node = Node(
        package = 'meldog_leg_tests',
        executable = 'demo_single_motor',
        parameters = [{
            'amplitude': AMPLITUDE,
            'which_motor': CHOSEN_MOTOR,
            'angular_velocity': ANGULAR_VELOCITY
        }],
        output = 'screen'
    )

    nodes = [ploter_node]
    ld = LaunchDescription(nodes)
    return ld
