from launch import LaunchDescription
from launch_ros.actions import Node



VARIABLE = "position"               # Type of variable (position, velocity or torque)
CHOSEN_MOTOR = 1                    # Motor whose data will be ploted (STARTING FROM 1, NOT 0!)
def generate_launch_description():
    ploter_node = Node(
        package = 'meldog_leg_tests',
        executable = 'moteus_ploter',
        parameters = [{
            'variable': VARIABLE,
            'which_motor': CHOSEN_MOTOR
        }],
        output = 'screen'
    )

    nodes = [ploter_node]
    ld = LaunchDescription(nodes)
    return ld
