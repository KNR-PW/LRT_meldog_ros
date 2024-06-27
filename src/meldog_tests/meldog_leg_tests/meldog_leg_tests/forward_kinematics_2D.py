import rclpy
from rclpy.node import Node
from meldog_interfaces.msg import MultiMoteusState
from geometry_msgs.msg import Vector3
import math
import matplotlib.pyplot as plt

class Leg_Forward_Kinematics_Solver(Node):

    def __init__(self, name):
        super().__init__(name)
        self.declare_parameter("length_1",0.225)
        self.declare_parameter("length_2",0.225)
        self.length_1 = self.get_parameter("length_1").value
        self.length_2 = self.get_parameter("length_2").value
        self.subscriber = self.create_subscription(MultiMoteusState,'multi_moteus_state',
                                                   self.forward_kinematics_callback,10)
        self.position_1 = 0.0
        self.position_2 = 0.0 # Katy obrotu w złączu (nie na silnikach!)

        self.publisher = self.create_publisher(Vector3, 'end_effector_actual_trajectory', 10)
        self.logger = self.get_logger()
        self.logger.info("Forward kinematics solver has started!")

    def forward_kinematics_solver(self):
        self.x = self.length_1*math.cos(self.position_1) + self.length_2*math.cos(self.position_1 + self.position_2)
        self.y = self.length_1*math.sin(self.position_1) + self.length_2*math.sin(self.position_1 + self.position_2)

    def forward_kinematics_callback(self,msg):
        self.position_1 = msg[0].position
        self.position_2 = msg[1].position - msg[0].position
        self.forward_kinematics_solver()
        end_effector_vector = Vector3()
        end_effector_vector.x = self.x
        end_effector_vector.y = self.y
        self.publisher.publish(end_effector_vector)


def main(args=None):
    rclpy.init(args=args)

    node = Leg_Forward_Kinematics_Solver("forward_kinematics_2D")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()