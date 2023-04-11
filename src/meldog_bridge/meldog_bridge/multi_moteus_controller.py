#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import asyncio
import moteus
from meldog_interfaces.msg import MultiMoteusControl
from meldog_interfaces.msg import MultiMoteusState
from meldog_interfaces.msg import MoteusState
from meldog_interfaces.msg import MoteusControl
from meldog_interfaces.srv import MultiMoteusActive

# TODO: Pomysl o serwerze do zatrzymywania moteusa w stalej pozycji/wylaczeniu?

class Multi_Moteus_Controller_Node(Node):
    async def __init__(self,name):
        super().__init__(name)
        # Parametry:

        self.declare_parameter("number_of_moteuses", 1)
        self.amount_of_servos= self.get_parameter("number_of_servos").value
        self.moteus_control_array = MultiMoteusControl()
        self.moteus_state_array = MultiMoteusState()
        self.moteus_index_list = range(1,self.amount_of_servos+1)
        self.active_servos_list = [] 

        # Inicjalizacja moteusow:

        self.transport = moteus.Fdcanusb()
        self.servos = self.multi_moteus_spawn()

        # Inicjalizacja publisherow, subscriberow i serwera serwisu:

        self.state_publisher_= self.create_publisher(MultiMoteusState, "multi_moteus_state", 10)
        self.control_subscriber_ = self.create_subscription(MultiMoteusControl, "multi_moteus_control",
                                                           self.control_callback, 10)
        # self.active_server_ = self.create_service(MultiMoteusActive, "multi_moteus_active", self.active_server_callback)
        self.timer_ = self.create_timer(0.01,self.state_publish)

    # Funkcja do publikacji stanu moteusow:

    def state_publish(self):
        msg = self.multi_moteus_state(self.moteus_state_array.state_array)
        self.state_publisher_.publish(msg)
    
    # Funkcja do odbierania polecen do moteusow:

    def control_callback(self, msg):
        self.moteus_control_array = msg
        self.multi_moteus_control(self.moteus_control_array.control_array)


    # Funkcja  do uzyskiwania stanow moteusow:

    def multi_moteus_state(self, state_array: MultiMoteusState.state_array):
        for id in self.moteus_index_list:
            state_array[id].position = self.results[id].values[moteus.Register.POSITION]
            state_array[id].velocity = self.results[id].values[moteus.Register.VELOCITY]
            state_array[id].torque = self.results[id].values[moteus.Register.TORQUE]
            state_array[id].q_current = self.results[id].values[moteus.Register.Q_CURRENT]
            state_array[id].d_current = self.results[id].values[moteus.Register.D_CURRENT]
        return state_array
    
    #Funkcja do generowania polecen do moteusow:

    async def multi_moteus_control(self, control_array: MultiMoteusControl.control_array):
        commands = [self.servos[id].set_position(position = control_array[id].desired_position, 
                                                 velocity = control_array[id].desired_velocity,
                                                 feedforward_torque = control_array[id].feedforward_torque,
                                                 query = True) 
                                                 for id in self.moteus_index_list]
        self.results = await self.transport.cycle(commands)    
    
    async def multi_moteus_spawn(self):
        self.servos = {servo_id: moteus.Controller(id = servo_id, transport = self.transport) 
                  for servo_id in self.moteus_index_list}
        await self.transport.cycle(servo.set_stop() for servo in self.servos.values())
    





async def main(args = None):
    rclpy.init(args = args)
    node  = Multi_Moteus_Controller_Node("multi_moteus_controller")
    rclpy.spin(node)
    rclpy.shutdown()





asyncio.run(main())