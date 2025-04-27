#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from meldog_interfaces_tests.msg import MultiMoteusControl 
from meldog_interfaces_tests.msg import MoteusControl
from geometry_msgs.msg import Vector3
import math
from rclpy.duration import Duration

class Demo_Single_Motor(Node):
    def __init__(self, name):
        super().__init__(name)
        self.declare_parameter("which_motor", 1)
        self.declare_parameter("angular_velocity", 1.0)
        self.declare_parameter("amplitude", 1.0)
        self.logger = self.get_logger()
        self.angular_velocity = self.get_parameter("angular_velocity").value
        self.amplitude = self.get_parameter("amplitude").value
        self.motor_index = self.get_parameter("which_motor").value
        self.time = 0
        self.timer_period = 0.001
        self.control_array = []
        self.control_array.append(MoteusControl())
        self.multi_moteus_control_msg = MultiMoteusControl()

        self.publisher = self.create_publisher(MultiMoteusControl,'multi_moteus_control',10);
        
        self.timer = self.create_timer(self.timer_period, self.sinus_callback)
        self.logger.info("Motor should start moving now!")
    def sinus_callback(self):
        self.control_array[self.motor_index - 1].desired_position = self.amplitude *math.sin(self.angular_velocity*self.time)
        self.control_array[self.motor_index - 1].desired_velocity = 0.0
        self.control_array[self.motor_index - 1].feedforward_torque = 0.0
        self.multi_moteus_control_msg.control_array = self.control_array
        self.publisher.publish(self.multi_moteus_control_msg)
        self.time += self.timer_period


def main(args=None):
    rclpy.init(args=args)

    node = Demo_Single_Motor("demo_single_motor")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
