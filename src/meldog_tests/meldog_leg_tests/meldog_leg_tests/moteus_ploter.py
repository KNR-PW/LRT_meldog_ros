import rclpy
from rclpy.node import Node
from meldog_interfaces_tests.msg import MultiMoteusControl, MultiMoteusState
from meldog_interfaces_tests.msg import MoteusControl, MoteusState
from geometry_msgs.msg import Vector3
import math
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import threading
import sys
class Ploter(Node):

    def __init__(self, name, plt: plt):
        super().__init__(name)
        self.logger = self.get_logger()
        self.declare_parameter("variable", "position")
        self.declare_parameter("which_motor", 1)
        self.variable = self.get_parameter("variable").value
        self.which_motor = self.get_parameter("which_motor").value
        self.y_label = ""
        if(self.variable == "position"):
             self.y_label = "Pozycja złącza [rad]"
        elif(self.variable == "velocity"):
             self.y_label = "Prędkość kątowa złącza [rad/s]"
        elif(self.variable == "torque"):
             self.y_label = "Moment złącza [Nm]"
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
        
        self.logger.info("Ploter initialized!")
    def plot_control(self, msg):
            self.control_value = msg.control_array[self.which_motor - 1].desired_position
        
    def plot_state(self, msg):       
            self.state_value = msg.state_array[self.which_motor - 1].position
    def update_plot(self):
         self.state_array.append(self.state_value)
         self.control_array.append(self.control_value)

         self.relative_error = abs((self.control_value-self.state_value)/(self.control_value+0.001))
         self.relative_error_array.append(self.relative_error)

    def animate_2(self, i, data_list_1, data_list_2):
        data_list_1 = data_list_1[-100:]
        data_list_2 = data_list_2[-100:]
        if(data_list_1 is None and data_list_2 is None):
             max_value = 1
             min_value = -1
        else:
            max_value = max([max(data_list_1),max(data_list_2)])
            min_value = min([min(data_list_1),min(data_list_2)])
            if(max_value >= 0):
                 max_value *= 1.2
            else:
                 max_value *= 0.8
            if(min_value <= 0):
                 min_value *= 1.2
            else:
                 min_value *= 0.8
        self.ax.clear()
        self.ax.set_ylim([min_value, max_value])
        self.ax.set_ylabel(self.y_label )
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
    ani_2 = animation.FuncAnimation(node.figure, node.animate_2, fargs=(node.control_array, node.state_array), interval = 50)
    plt.show()

    


if __name__ == '__main__':
    main()


