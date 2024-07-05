#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from meldog_interfaces_tests.msg import MultiMoteusControl
from meldog_interfaces_tests.msg import MoteusControl
from geometry_msgs.msg import Vector3
import math
import matplotlib.pyplot as plt

class Leg_Inverse_Kinematics_Solver(Node):

    def __init__(self, name):
        super().__init__(name)

        self.declare_parameter("length_1",0.225)
        self.declare_parameter("length_2",0.225)
        self.declare_parameter("start_position", [0.0, -0.35])
        self.length_1 = self.get_parameter("length_1").value
        self.length_2 = self.get_parameter("length_2").value
        self.end_effector_vector = self.get_parameter("start_position").value

        self.position = [0,0]
        
        self.multi_moteus_control_msg = MultiMoteusControl()
        self.control_array = []
        for id in range(2):
            self.control_array.append(MoteusControl())

        self.publisher = self.create_publisher(MultiMoteusControl,'multi_moteus_control',10)

        timer_period = 0.001
        self.timer = self.create_timer(timer_period, self.inverse_kinematics_callback)

        self.subscription = self.create_subscription(Vector3,"end_effector_desired_trajectory",self.listener_callback,10)
        
        self.logger = self.get_logger()
        self.logger.info("Inverse kinematics solver has started!")
    def inverse_kinematics_solver(self):
        x = self.end_effector_vector[0]
        y = self.end_effector_vector[1]
        
        w = -math.acos((x**2+y**2-self.length_1**2-self.length_2**2)/(2*self.length_1*self.length_2))
        e = self.length_1 + self.length_2*math.cos(w)
        f = self.length_2*math.sin(w)
        cos_q1 = (f*y + e*x)/(f**2 + e**2)
        sin_q1 = (e*cos_q1 - x)/f
        self.position[0] = math.atan2(sin_q1,cos_q1) + math.pi/2
        self.position[1]= self.position[0] + w

    def inverse_kinematics_callback(self):
        self.inverse_kinematics_solver()
        
        for id in range(2):
            self.control_array[id].desired_position = self.position[id]
            self.control_array[id].desired_velocity = 0.0
            self.control_array[id].feedforward_torque = 0.0
            self.multi_moteus_control_msg.control_array = self.control_array
        self.publisher.publish(self.multi_moteus_control_msg)
    

    def listener_callback(self,msg):
        self.end_effector_vector[0] = msg.x
        self.end_effector_vector[1] = msg.y

    
        

def main(args=None):
    rclpy.init(args=args)

    node = Leg_Inverse_Kinematics_Solver("inverse_kinematics_2D")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
