#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import asyncio
import moteus
from meldog_interfaces_tests.msg import MultiMoteusControl, MultiMoteusState, MoteusState, MoteusControl
import sys
import math
import threading
import moteus.multiplex
import copy
import queue

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
        self.i = 0
        self.results = []

        # Czy doszlo do komunikacji na kanale control:

        self.current_request = False

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

        # Kolejka sygnalow sterujacych dla moteusow:

        self.control_queue = queue.Queue()

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

        # Inicjalizacja moteusow:
         
        self.multi_moteus_init()

        # Petla sterowania moteusami:

        while True:
            
            # with self.lock:
            if not self.control_queue.empty():
                self.current_control_msg = self.control_queue.get()
                target = self.current_request
            #print((self.state_arrays[0].position-self.current_control_msg.control_array[0].desired_position)/(self.state_arrays[0].position+0.01))
            await self.multi_moteus_control(self.current_control_msg.control_array)
            self.state_publish()
            
            # else:
            #     await self.multi_moteus_query()
            #     print("Odczytuje")
            #     await asyncio.sleep(0.01)

    # Funkcja do publikacji stanu moteusow:

    def state_publish(self):
        self.multi_moteus_state()
        state_procent = abs((self.state_arrays[0].position-self.current_control_msg.control_array[0].desired_position)/self.state_arrays[0].position+0.001)
        print("\r Dokladnosc: " + "{:.2f}".format(state_procent) + " % " + "Moment: " + "{:.2f}".format(abs(self.state_arrays[0].torque)) + " Nm" + 
              " Queue size: " + str(self.control_queue.qsize()),end = "\r",flush = True)
        msg = self.multi_moteus_state_msg
        self.state_publisher_.publish(msg)

    # Funkcja do odbierania polecen do moteusow:

    def control_callback(self, msg):
        # with self.lock:
        if not self.same_messages(self.current_control_msg.control_array, msg.control_array):
            self.control_queue.put(msg)
            self.current_request = True

    # Funkcja  do uzyskiwania stanow moteusow:

    def multi_moteus_state(self): 
        for id in self.moteus_index_list:
            self.state_arrays[id-1].position = self.results[id-1].values[moteus.Register.POSITION]*2*math.pi/16
            self.state_arrays[id-1].velocity = self.results[id-1].values[moteus.Register.VELOCITY]*2*math.pi
            self.state_arrays[id-1].torque = self.results[id-1].values[moteus.Register.TORQUE]
            self.state_arrays[id-1].q_current = self.results[id-1].values[moteus.Register.Q_CURRENT]
            self.state_arrays[id-1].d_current = self.results[id-1].values[moteus.Register.D_CURRENT]
        self.multi_moteus_state_msg.state_array = self.state_arrays

    # Funkcja do generowania polecen do moteusow:

    async def multi_moteus_control(self, control_array: MultiMoteusControl.control_array):
        commands = [self.servos[id].make_position(position=control_array[id-1].desired_position/(2*math.pi)*16,
                                                 velocity= 0.0,
                                                 feedforward_torque=0.0, 
                                                 #velocity_limit = 150/(2*math.pi),
                                                 maximum_torque = 0.3,
                                                 #accel_limit = 2500/(2*math.pi),
                                                 query=True)
                    for id in self.moteus_index_list]
        self.results = await self.transport.cycle(commands)
        #await asyncio.sleep(0.0001)

    # Funkcja do restartu moteusow:

    async def multi_moteus_spawn(self):
        commands = [servo.make_stop(query = True) for servo in self.servos.values()]
        await self.transport.cycle(commands)

    # Funkcja inicjalizacji pozycji (aby moteusy mogly wysylac sygnaly):

    def multi_moteus_init(self):
        init_msg = MultiMoteusControl()
        init_list = []
        single_msg = MoteusControl()
        single_msg.desired_position = 0.0
        single_msg.desired_velocity = 0.0
        single_msg.feedforward_torque = 0.0

        for id in self.moteus_index_list:
            init_list.append(single_msg)
        init_msg.control_array = init_list    
        self.control_queue.put(init_msg)

    # Funkcja pozyskujaca aktualne pomiary dla moteusa:

    async def multi_moteus_query(self):
        commands = [servo.make_query()
                        for servo in self.servos.values()]
        self.results = await self.transport.cycle(commands)
        await asyncio.sleep(0.01)
    
    def same_messages(self, current_message_array: MultiMoteusControl.control_array, new_message_array: MultiMoteusControl.control_array):
        all_same = True
        for id in self.moteus_index_list:
            current_same = current_message_array[id-1].desired_position == new_message_array[id-1].desired_position and current_message_array[id-1].desired_velocity == new_message_array[id-1].desired_velocity and current_message_array[id-1].feedforward_torque == new_message_array[id-1].feedforward_torque
            all_same  = all_same and current_same
        return all_same



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
        # node.timer_ = node.create_timer(0.01, node.state_publish)
        rclpy.spin(node)
        rclpy.shutdown()
    
    ros_thread = threading.Thread(target=start_ros)
    ros_thread.start()

    await node.run_controller()
def main():
    asyncio.run(main_coroutine())

if __name__ == "__main__":
    main()
