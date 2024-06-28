from launch import LaunchDescription
from launch_ros.actions import Node



NUMBER_OF_SERVOS = 2
GEAR_RATIOS = 16
def generate_launch_description():
    controller_node = Node(
        package = 'meldog_leg_tests',
        executable = 'multi_moteus_controller_single_thread',
        parameters = [{
            'number_of_servos': NUMBER_OF_SERVOS,
            'gear_ratio': GEAR_RATIOS,
        }],
        output = 'screen'
    )

    nodes = [controller_node]
    ld = LaunchDescription(nodes)
    return ld
