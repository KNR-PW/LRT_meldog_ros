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
        self.timer = self.create_timer(0.01, self.update_plot)

        self.time_list = []
        self.control_array = []
        self.state_array = []
        self.control_value = 0
        self.state_value = 0
        self.relative_error = 0
        self.relative_error_array = []
        self.figure = plt.figure()
        self.ax = self.figure.add_subplot(111)
        

    def plot_control(self, msg):
            self.control_value = msg.control_array[0].feedforward_torque
        
    def plot_state(self, msg):       
            self.state_value = msg.state_array[0].torque
    def update_plot(self):
         self.state_array.append(self.state_value)
         self.control_array.append(self.control_value)

         self.relative_error = abs((self.control_value-self.state_value)/(self.control_value+0.001))
         self.relative_error_array.append(self.relative_error)

    def animate_2(self, i, data_list_1, data_list_2):
        data_list_1 = data_list_1[-100:]
        data_list_2 = data_list_2[-100:]
        self.ax.clear()
        self.ax.set_ylim([-20, 20])
        self.ax.set_ylabel("Pozycja [rad]")
        self.ax.plot(data_list_1, label = "ROS")
        self.ax.plot(data_list_2, label = "Silnik")
        self.ax.legend()
    def animate_1(self, i, data_list):
        data_list = data_list[-100:]
        self.ax.clear()
        self.ax.set_ylim([-2, 2])
        self.ax.set_ylabel("Blad wzgledny")
        self.ax.plot(data_list)
    
def main(args=None):
    rclpy.init(args=args)
    node = Ploter("Ploter_node", plt)
    def start_ros():
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    
    ros_thread = threading.Thread(target=start_ros)
    ros_thread.start()
    #ani_1 = animation.FuncAnimation(node.figure, node.animate_1, fargs=(node.relative_error_array), interval = 50)
    ani_2 = animation.FuncAnimation(node.figure, node.animate_2, fargs=(node.control_array, node.state_array), interval = 10)
    plt.show()

    


if __name__ == '__main__':
    main()


