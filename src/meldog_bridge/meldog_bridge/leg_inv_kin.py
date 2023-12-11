#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from meldog_interfaces.msg import MultiMoteusControl
from meldog_interfaces.msg import MoteusControl
from geometry_msgs.msg import Vector3
import math
import matplotlib.pyplot as plt

class Leg_Inverse_Kinematics_Solver(Node):

    def __init__(self, name):
        super().__init__(name)

        self.declare_parameter("length_1",0.25)
        self.declare_parameter("length_2",0.25)
        self.declare_parameter("gear_ratio",16)
        self.length_1 = self.get_parameter("length_1").value
        self.length_2 = self.get_parameter("length_2").value
        self.gear_ratio = self.get_parameter("gear_ratio").value
        self.position = [0,0]
        self.end_effector_vector = [0.0, 0.35]

        self.multi_moteus_control_msg = MultiMoteusControl()
        self.control_array = []
        for id in range(2):
            self.control_array.append(MoteusControl())

        self.publisher = self.create_publisher(MultiMoteusControl,'multi_moteus_control',10);

        timer_period = 0.015
        self.timer = self.create_timer(timer_period, self.inverse_kinematics_callback)

        self.subscription = self.create_subscription(Vector3,"end_effector_trajectory",self.listener_callback,10)

    def inverse_kinematics_solver(self):
        x = self.end_effector_vector[0]
        y = self.end_effector_vector[1]

        w = -math.acos((x**2+y**2-self.length_1**2-self.length_2**2)/(2*self.length_1*self.length_2))
        a = ((self.length_1*math.cos(w)+self.length_2)*y - self.length_1*math.sin(w)*x)/(x**2+y**2)
        b = -((self.length_1*math.cos(w)+self.length_2)*x + self.length_1*math.sin(w)*y)/(x**2+y**2)
        self.position[1] = math.atan2(b,a)
        self.position[0]= self.position[1] + w

        # x_new = self.length_1*math.cos(self.position[0]+math.pi/2) + self.length_2*math.cos(self.position[1]+math.pi/2)
        # y_new = self.length_1*math.sin(self.position[0]+math.pi/2) + self.length_2*math.sin(self.position[1]+math.pi/2)
        # print(x_new)
        # print(y_new)
        

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

    node = Leg_Inverse_Kinematics_Solver("leg_inv_solver")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
