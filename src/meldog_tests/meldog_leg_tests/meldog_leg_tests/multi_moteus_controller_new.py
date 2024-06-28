#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
import asyncio
import moteus
from meldog_interfaces.msg import MultiMoteusControl, MultiMoteusState, MoteusState, MoteusControl
from meldog_interfaces.srv import MultiMoteusActive
import sys
import math
import moteus.multiplex


# TODO: Dodaj serwer do wylaczenia moteusa (make_stop())


class Multi_Moteus_Controller_Node(Node):
    def __init__(self, name):
        super().__init__(name)
        # Parametry:

        # Liczba moteusow:

        self.declare_parameter("number_of_servos", 2) 
        self.amount_of_servos = self.get_parameter("number_of_servos").value  
        self.declare_parameter("gear_ratio", 16)
        self.gear_ratio = self.get_parameter("gear_ratio")
        self.moteus_index_list = range(1, self.amount_of_servos+1)
        self.clock = self.get_clock()
        self.logger = self.get_logger()
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

        # Laczenie z moteusami:

        try:
            self.transport = moteus.Fdcanusb()
        except RuntimeError:
            self.logger.error("Cannot connect to moteuses!")
            sys.exit()
        self.servos = {servo_id: moteus.Controller(id=servo_id, transport=self.transport)
                       for servo_id in self.moteus_index_list}
        self.logger.info("Successfully connected to moteuses!")
        #Inicjalizacja publisherow i subscriberow:
        
        self.state_publisher_ = self.create_publisher(
            MultiMoteusState, "multi_moteus_state", 10)
        self.control_subscriber_ = self.create_subscription(MultiMoteusControl, "multi_moteus_control",
                                                            self.control_callback, qos_profile = 1)
        
    async def start_controller(self):
        # Restart moteusow:

        await self.multi_moteus_spawn()

        # Inicjalizacja moteusow:
         
        await self.multi_moteus_init()

    async def run_controller(self):
            await self.multi_moteus_control(self.multi_moteus_control_msg.control_array)
            self.state_publish()
            
    # Funkcja do publikacji stanu moteusow:

    def state_publish(self):
        self.multi_moteus_state()
        self.state_publisher_.publish(self.multi_moteus_state_msg)

    # Funkcja do odbierania polecen do moteusow:

    def control_callback(self, msg):
        self.multi_moteus_control_msg = msg
        

    # Funkcja  do uzyskiwania stanow moteusow:

    def multi_moteus_state(self): 
        for id in self.moteus_index_list:
            self.state_arrays[id-1].position = self.results[id-1].values[moteus.Register.POSITION]*2*math.pi/self.gear_ratio
            self.state_arrays[id-1].velocity = self.results[id-1].values[moteus.Register.VELOCITY]*2*math.pi/self.gear_ratio
            self.state_arrays[id-1].torque = self.results[id-1].values[moteus.Register.TORQUE]*self.gear_ratio
            self.state_arrays[id-1].q_current = self.results[id-1].values[moteus.Register.Q_CURRENT]
            self.state_arrays[id-1].d_current = self.results[id-1].values[moteus.Register.D_CURRENT]
        self.multi_moteus_state_msg.state_array = self.state_arrays

    # Funkcja do generowania polecen do moteusow:

    async def multi_moteus_control(self, control_array: MultiMoteusControl.control_array):
        
        commands = [self.servos[id].make_position(position=control_array[id-1].desired_position/(2*math.pi)*self.gear_ratio,
                                                 velocity= 0.0,
                                                 feedforward_torque=control_array[id-1].feedforward_torque/self.gear_ratio, 
                                                 #velocity_limit = 150/(2*math.pi),
                                                 maximum_torque = 0.2,
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

    async def multi_moteus_init(self):

        commands = [self.servos[id].make_position(position=0.0,
                                                 velocity= 0.0,
                                                 feedforward_torque=0.0, 
                                                 velocity_limit = 30/(2*math.pi),
                                                 maximum_torque = 0.1,
                                                 accel_limit = 100/(2*math.pi),
                                                 query=True)
                    for id in self.moteus_index_list]
        await self.transport.cycle(commands)
        self.clock.sleep_for(Duration(nanoseconds = 10**6))    
        init_msg = MultiMoteusControl()
        init_list = []
        single_msg = MoteusControl()
        single_msg.desired_position = 0.0
        single_msg.desired_velocity = 0.0
        single_msg.feedforward_torque = 0.0

        for id in self.moteus_index_list:
            init_list.append(single_msg)
        init_msg.control_array = init_list
        self.multi_moteus_control_msg = init_msg
    # Funkcja pozyskujaca aktualne pomiary dla moteusa:

    async def multi_moteus_query(self):
        commands = [servo.make_query()
                        for servo in self.servos.values()]
        self.results = await self.transport.cycle(commands)
        #await asyncio.sleep(0.01)



async def main_coroutine():
    rclpy.init()
    control_node = Multi_Moteus_Controller_Node("multi_moteus_controller")
    await control_node.start_controller()      
    while(rclpy.ok()):
        await control_node.run_controller()
        rclpy.spin_once(node  = control_node, timeout_sec=0)

    rclpy.shutdown()
def main():
    asyncio.run(main_coroutine())

if __name__ == "__main__":
    main()
