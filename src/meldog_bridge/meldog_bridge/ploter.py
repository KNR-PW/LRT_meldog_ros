import rclpy
from rclpy.node import Node
from meldog_interfaces.msg import MultiMoteusControl, MultiMoteusState
from meldog_interfaces.msg import MoteusControl, MoteusState
from geometry_msgs.msg import Vector3
import math
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import threading

class Ploter(Node):

    def __init__(self, name, plt: plt):
        super().__init__(name)

        self.control_subscriber = self.create_subscription(MultiMoteusControl,'multi_moteus_control',self.plot_control,10)
        self.state_subscriber = self.create_subscription(MultiMoteusState,'multi_moteus_state',self.plot_state,10)
        self.time_list = []
        self.control_array = []
        self.state_array = []
        self.control_value = 0
        self.state_value = 0
        self.control_counter = 0
        self.state_counter = 0

        self.figure = plt.figure()
        self.ax = self.figure.add_subplot(111)
        

    def plot_control(self, msg):
            self.control_value = msg.control_array[0].desired_position
            self.control_array.append(self.control_value)
        
    def plot_state(self, msg):
            self.state_value = msg.state_array[0].position
            self.state_array.append(self.state_value)

    def animate(self, i, time_list, data_list_1, data_list_2):
        data_list_1 = data_list_1[-50:]
        data_list_2 = data_list_2[-50:]
        self.ax.clear()
        self.ax.set_xlabel("Czas [s]")
        self.ax.set_ylabel("Pozycja [rad]")
        self.ax.set_ylim([-25, 25])
        self.ax.plot(data_list_1, label = "ROS")
        self.ax.plot(data_list_2, label = "Silnik")
        self.ax.legend()

def main(args=None):
    rclpy.init(args=args)
    node = Ploter("Ploter_node", plt)
    def start_ros():
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    
    ros_thread = threading.Thread(target=start_ros)
    ros_thread.start()
    ani = animation.FuncAnimation(node.figure, node.animate, fargs=(node.time_list, node.control_array, node.state_array), interval = 100)
    plt.show()

    


if __name__ == '__main__':
    main()


