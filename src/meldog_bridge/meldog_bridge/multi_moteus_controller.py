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
import sys
import math
import threading
import moteus.multiplex

# TODO: Dodaj serwer do wylaczenia moteusa (make_stop())


class Multi_Moteus_Controller_Node(Node):
    def __init__(self, name):
        super().__init__(name)
        # Parametry:

        # Liczba moteusow:

        self.declare_parameter("number_of_servos", 1) 
        self.amount_of_servos = self.get_parameter("number_of_servos").value  
        self.moteus_index_list = range(1, self.amount_of_servos+1)

        # Wiadomosci subscribera i publishera:

        self.multi_moteus_control_msg = MultiMoteusControl()
        self.multi_moteus_state_msg = MultiMoteusState()

        # Wyniki pomiaru moteusow:

        self.results = []

        # Lista stanow moteusow:

        self.state_arrays = []
        for id in self.moteus_index_list:
            self.state_arrays.append(MoteusState())

        # Dodanie pomiarow Q_current i D_current do wartosci mierzonych:

        moteus_query_resolution = moteus.QueryResolution
        moteus_query_resolution.q_current = moteus.multiplex.F32
        moteus_query_resolution.d_current = moteus.multiplex.F32

        # Tworzenie watku do sterowania moteusami:

        self.lock = threading.Lock()
        self.current_request = None

        # Laczenie z moteusami:

        try:
            self.transport = moteus.Fdcanusb()
        except RuntimeError:
            print("No moteus detected")
            sys.exit()
        self.servos = {servo_id: moteus.Controller(id=servo_id, transport=self.transport)
                       for servo_id in self.moteus_index_list}
        

        

    async def run_controller(self):
        # Restart moteusow:

        await self.multi_moteus_spawn()
        await asyncio.sleep(0.01)

        # Inicjalizacja moteusow:
         
        await self.multi_moteus_init()
        await asyncio.sleep(0.01)

        # Petla sterowania moteusami:

        while True:
            with self.lock:
                target = self.current_request

            if target:
                await self.multi_moteus_control(self.multi_moteus_control_msg.control_array)
                self.current_request = False
            else:
                await self.multi_moteus_query()
            await asyncio.sleep(0.001)

    # Funkcja do publikacji stanu moteusow:

    def state_publish(self):
        self.multi_moteus_state()
        msg = self.multi_moteus_state_msg
        self.state_publisher_.publish(msg)

    # Funkcja do odbierania polecen do moteusow:

    def control_callback(self, msg):
        with self.lock:
            self.multi_moteus_control_msg = msg
            self.current_request = True

    # Funkcja  do uzyskiwania stanow moteusow:

    def multi_moteus_state(self): 
        for id in self.moteus_index_list:
            self.state_arrays[id-1].position = self.results[id-1].values[moteus.Register.POSITION]
            self.state_arrays[id-1].velocity = self.results[id-1].values[moteus.Register.VELOCITY]
            self.state_arrays[id-1].torque = self.results[id-1].values[moteus.Register.TORQUE]
            self.state_arrays[id-1].q_current = self.results[id-1].values[moteus.Register.Q_CURRENT]
            self.state_arrays[id-1].d_current = self.results[id-1].values[moteus.Register.D_CURRENT]
        self.multi_moteus_state_msg.state_array = self.state_arrays

    # Funkcja do generowania polecen do moteusow:

    async def multi_moteus_control(self, control_array: MultiMoteusControl.control_array):
        commands = [self.servos[id].make_position(position=control_array[id-1].desired_position,
                                                 velocity=control_array[id-1].desired_velocity,
                                                 feedforward_torque=control_array[id-1].feedforward_torque, velocity_limit = 20,
                                                 query=True)
                    for id in self.moteus_index_list]
        print("Pozycja:" + str(control_array[0].desired_position))
        self.results = await self.transport.cycle(commands)

    # Funkcja do restartu moteusow:

    async def multi_moteus_spawn(self):
        commands = [servo.make_stop(query = True) for servo in self.servos.values()]
        await self.transport.cycle(commands)

    # Funkcja inicjalizacji pozycji (aby moteusy mogly wysylac sygnaly):

    async def multi_moteus_init(self):
        commands = [servo.make_position(position= 0, velocity_limit = 1, query=True)
                    for servo in self.servos.values()]
        self.results = await self.transport.cycle(commands)

    # Funkcja pozyskujaca aktualne pomiary dla moteusa:

    async def multi_moteus_query(self):
        commands = [servo.make_query()
                    for servo in self.servos.values()]
        self.results = await self.transport.cycle(commands)



async def main_coroutine():
    rclpy.init()
    node = Multi_Moteus_Controller_Node("multi_moteus_controller")
    def start_ros():
        # Inicjalizacja publisherow, subscriberow i serwera serwisu:

        node.state_publisher_ = node.create_publisher(
            MultiMoteusState, "multi_moteus_state", 10)
        node.control_subscriber_ = node.create_subscription(MultiMoteusControl, "multi_moteus_control",
                                                            node.control_callback, 10)
        # self.active_server_ = self.create_service(MultiMoteusActive, "multi_moteus_active", self.active_server_callback)
        node.timer_ = node.create_timer(0.1, node.state_publish)
        rclpy.spin(node)
        rclpy.shutdown()
    
    ros_thread = threading.Thread(target=start_ros)
    ros_thread.start()

    await node.run_controller()
def main():
    asyncio.run(main_coroutine())

if __name__ == "__main__":
    main()
