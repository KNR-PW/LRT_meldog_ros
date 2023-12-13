#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from meldog_interfaces.msg import MoteusControl, MultiMoteusControl, MoteusState, MultiMoteusState
from geometry_msgs.msg import Vector3
import math
import pandas 
import numpy

class Optimal_Jump_Controler_Node(Node):

    def __init__(self, name):
        super().__init__(name)

        self.PREPARE_FOR_FIRST_JUMP = 0
        self.FIRST_JUMP = 1
        self.FLY = 2
        self.JUMP = 3
        self.state = self.PREPARE_FOR_FIRST_JUMP
        self.STOP = 4

        self.multi_moteus_control_msg = MultiMoteusControl()
        self.multi_moteus_state_msg = ()

        self.publisher = self.create_publisher(MultiMoteusControl,'multi_moteus_control',10)
        self.subscription = self.create_subscription(MultiMoteusState,"multi_moteus_state",self.listener_callback,10)


        df = pandas.read_csv('jump_data.csv')

        # PIERWSZY SKOK
        self.first_jump_time = df['time_first_jump'].to_numpy()
        self.first_jump_position_1 = df['alfa_first_jump'].to_numpy()
        self.first_jump_torque_1 = df['t1_first_jump'].to_numpy()
        self.first_jump_position_2 = df['beta__first_jump'].to_numpy()
        self.first_jump_torque_2 = df['t2__first_jump'].to_numpy()

        self.first_jump_time = self.first_jump_time[~numpy.isnan(self.first_jump_time)]
        self.first_jump_position_1 = self.first_jump_position_1[~numpy.isnan(self.first_jump_position_1)]
        self.first_jump_torque_1 = self.first_jump_torque_1[~numpy.isnan(self.first_jump_torque_1)]
        self.first_jump_position_2 = self.first_jump_position_2[~numpy.isnan(self.first_jump_position_2)] 
        self.first_jump_torque_2 = self.first_jump_torque_2[~numpy.isnan(self.first_jump_torque_2)]

        # FAZA LOTU
        self.fly_time = df['time_fly'].to_numpy()
        self.fly_position_1 = df['alfa_fly'].to_numpy()
        self.fly_torque_1 = df['t1_fly'].to_numpy()
        self.fly_position_2 = df['beta_fly'].to_numpy()
        self.fly_torque_2 = df['t2_fly'].to_numpy()

        self.fly_time = self.fly_time[~numpy.isnan(self.fly_time)]
        self.fly_position_1 = self.fly_position_1[~numpy.isnan(self.fly_position_1)]
        self.fly_torque_1 = self.fly_torque_1[~numpy.isnan(self.fly_torque_1)]
        self.fly_position_2 = self.fly_position_2[~numpy.isnan(self.fly_position_2)] 
        self.fly_torque_2 = self.fly_torque_2[~numpy.isnan(self.first_jump_torque_2)]

        # FAZA UPADKU I SKOKU
        self.jump_time = df['time_jump'].to_numpy()
        self.jump_position_1 = df['alfa_jump'].to_numpy()
        self.jump_torque_1 = df['t1_jump'].to_numpy()
        self.jump_position_2 = df['beta_jump'].to_numpy()
        self.jump_torque_2 = df['t2_jump'].to_numpy()

        self.jump_time = self.jump_time[~numpy.isnan(self.jump_time)]
        self.jump_position_1 = self.jump_position_1[~numpy.isnan(self.jump_position_1)]
        self.jump_torque_1 = self.jump_torque_1[~numpy.isnan(self.jump_torque_1)]
        self.jump_position_2 = self.jump_position_2[~numpy.isnan(self.jump_position_2)] 
        self.jump_torque_2 = self.jump_torque_2[~numpy.isnan(self.first_jump_torque_2)]
    
        # PREDKOSCI KRYTYCZNE (OD NICH WIEMY KIEDY UPADNIE PRZED CZASEM) 
        self.critical_velocities = df['dqdt_critical'].to_numpy()
        self.critical_velocities = self.critical_velocities[~numpy.isnan(self.critical_velocities)]
        self.critital_velocities_factor = 1

        timer_period = 0.0025
        self.timer = self.create_timer(timer_period, self.control_publisher)

        self.clock = self.get_clock()

        while(True):
            self.jumping_control()

    def publish_control(self):
        self.publisher.publish(self.multi_moteus_control_msg)

    def listener_callback(self, msg: MultiMoteusState):
        self.multi_moteus_state_msg = msg 

    def jumping_control(self):
        if(self.state == self.PREPARE_FOR_FIRST_JUMP):
            self.prepare_first_jump()
        elif(self.state == self.FIRST_JUMP):
            self.first_jump_control()
        elif(self.state == self.FLY):
            self.flying_control()
        elif(self.state == self.JUMP):
            self.jump_control()
        
     
    def prepare_first_jump(self):
        print("START!")
        actual_pisition_1 = self.multi_moteus_state_msg.state_array[0].position
        actual_pisition_2 = self.multi_moteus_state_msg.state_array[0].position
        desired_position_1 = self.first_jump_position_1[0]
        desired_position_2 = self.first_jump_position_2[0]
        while(not(math.iscloe(desired_position_1,actual_pisition_1,abs_tol = 0.01) and math.iscloe(desired_position_2,actual_pisition_2,abs_tol = 0.01))):
            self.change_control_message(desired_position_1, desired_position_2)
            self.publish_control()
            self.clock.sleep_for(Duration(nanoseconds = 10**6))

        self.state = self.FIRST_JUMP
        self.clock.sleep_for(Duration(nanoseconds = 10**9))
        return

    def first_jump_control(self):
        print("FIRST JUMP!")
        i = 0
        while(i < (len(self.first_jump_time)-1)):
            self.change_control_message(self.first_jump_position_1[i], self.first_jump_position_2[i],
                                        self.first_jump_torque_1[i], self.first_jump_torque_2[i])
            self.publish_control()
            self.clock.sleep_for(Duration(nanoseconds = (self.first_jump_time[i+1]-self.first_jump_time[i])*10**9))
            i += 1
        self.change_control_message(self.first_jump_position_1[i], self.first_jump_position_2[i],
                                        self.first_jump_torque_1[i], self.first_jump_torque_2[i])
        self.publish_control()
        self.state = self.FLY
        return
    
    ## TODO: POMYSL CZY NIE ZMIENIC NA TO ABY CZEKAL NA SKOKU W PREDKOSCIACH KATOWYCH
    def flying_control(self):
        print("FLYING PHASE!")
        i = 0
        while(i < (len(self.fly_time)-1) and not self.leg_on_ground()):
            self.change_control_message(self.fly_position_1[i], self.fly_position_2[i],
                                        self.fly_torque_1[i], self.fly_torque_2[i])
            self.clock.sleep_for(Duration(nanoseconds = (self.fly_time[i+1]-self.fly_time[i])*10**9))
            self.publish_control()
            i += 1
        self.change_control_message(self.fly_position_1[i], self.fly_position_2[i],
                                        self.fly_torque_1[i], self.fly_torque_2[i])
        self.publish_control()
        if(not self.leg_on_ground()):
            self.clock.sleep_for(Duration(nanoseconds = 5*10**6))
        self.state = self.JUMP
        return
    
    def jump_control(self):
        print("JUMPING PHASE!")
        i = 0
        while(i < (len(self.jump_time)-1)):
            self.change_control_message(self.jump_position_1[i], self.jump_position_2[i],
                                        self.jump_torque_1[i], self.jump_torque_2[i])
            self.publish_control()
            self.clock.sleep_for(Duration(nanoseconds = (self.jump_time[i+1]-self.jump_time[i])*10**9))
            i += 1
        self.change_control_message(self.jump_position_1[i], self.jump_position_2[i], 
                                        self.jump_torque_1[i], self.jump_torque_2[i])
        self.publish_control()
        self.state = self.FLY
        return
    
    def change_control_message(self, position_1, position_2, velocity_1 = 0.0, velocity_2 = 0.0, torque_1 = 0.0, torque_2 = 0.0):
        self.multi_moteus_control_msg.control_array[0].desired_position = position_1
        self.multi_moteus_control_msg.control_array[0].desired_velocity = velocity_1
        self.multi_moteus_control_msg.control_array[0].feedforward_torque = torque_1
        self.multi_moteus_control_msg.control_array[1].desired_position = position_2
        self.multi_moteus_control_msg.control_array[1].desired_velocity = velocity_2
        self.multi_moteus_control_msg.control_array[1].feedforward_torque = torque_2

    def leg_on_ground(self):
        velocity_1 = self.multi_moteus_state_msg.state_array[0].velocity / self.critital_velocities_factor
        velocity_2 = self.multi_moteus_state_msg.state_array[1].velocity / self.critital_velocities_factor
        if(velocity_1>= self.critical_velocities[0] and velocity_2<= self.critical_velocities[1]):
            return True
        return False

def main(args=None):
    rclpy.init(args=args)

    node = Optimal_Jump_Controler_Node("optimal_jump_controler")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
