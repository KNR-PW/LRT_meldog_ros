#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from meldog_interfaces.msg import MultiMoteusControl
from meldog_interfaces.msg import MoteusControl
from geometry_msgs.msg import Vector3
import math
import matplotlib.pyplot as plt

class Demo_Single_Motor(Node):
    def __init__(self, name):
        super().__init__(name)

        self.frequency = 1/5
        self.amplitude = 8*math.pi
        self.time = 0
        self.timer_period = 0.01
        self.control_array = []
        self.control_array.append(MoteusControl())
        self.multi_moteus_control_msg = MultiMoteusControl()

        self.publisher = self.create_publisher(MultiMoteusControl,'multi_moteus_control',10);
        
        self.timer = self.create_timer(self.timer_period, self.sinus_callback)

    def sinus_callback(self):

        self.control_array[0].desired_position = self.amplitude *math.sin(2*math.pi*self.frequency*self.time)
        self.control_array[0].desired_velocity = 0.0
        self.control_array[0].feedforward_torque = 0.0
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
