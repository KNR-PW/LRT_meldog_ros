from launch import LaunchDescription
from launch_ros.actions import Node



FILENAME = "jump_data_St_nice.csv"          # Name of csv file (in jump_data/)
def generate_launch_description():
    jumping_node = Node(
        package = 'meldog_leg_tests',
        executable = 'jump_control_node',
        parameters = [{
            "filepath": FILENAME
        }],
        output = 'screen'
    )

    nodes = [jumping_node]
    ld = LaunchDescription(nodes)
    return ld
