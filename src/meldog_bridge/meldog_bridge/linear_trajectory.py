import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
import math

class Circle_Trajectory(Node):
    def __init__(self,name):
        super().__init__(name)
        self.declare_parameter("position_x", 0.0)
        self.declare_parameter("position_y", 0.3)
        self.declare_parameter("radius",0.15)
        self.declare_parameter("angular_velocity",3)
        self.x = self.get_parameter("position_x").value
        self.y = self.get_parameter("position_y").value
        self.r = self.get_parameter("radius").value
        self.w = self.get_parameter("angular_velocity").value

        self.t = 0

        self.end_effector_position = Vector3()

        self.publisher_ = self.create_publisher(Vector3, 'end_effector_trajectory', 10)
        self.timer_period = 0.005  # seconds
        self.timer = self.create_timer(self.timer_period, self.trajectory_callback)

    def trajectory_callback(self):
        self.end_effector_position.x = 0.0
        self.end_effector_position.y = self.y + self.r*math.sin(self.w*self.t)
        self.end_effector_position.z = 0.0
        self.t += self.timer_period
        self.publisher_.publish(self.end_effector_position)
        if(self.t > 2*math.pi/self.w):
            self.t = 0



def main(args=None):
    rclpy.init(args=args)
    node = Circle_Trajectory("circle_traj")
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()