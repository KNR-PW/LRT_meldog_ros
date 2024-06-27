import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
import math
from rclpy.duration import Duration
from rclpy.time import Time
import rclpy.time

class Linear_Trajectory(Node):
    def __init__(self,name):
        super().__init__(name)
        self.declare_parameter("start_position", [0.0, -0.35])
        self.declare_parameter("radius",0.125)
        self.declare_parameter("angular_velocity",1.0)
        self.x = self.get_parameter("start_position").value[0]
        self.y = self.get_parameter("start_position").value[1]
        self.radius = self.get_parameter("radius").value
        self.angular_velocity = self.get_parameter("angular_velocity").value

        self.logger = self.get_logger()

        self.clock = self.get_clock()
        self.time_prev = self.clock.now()


        period_seconds = math.floor(2*math.pi/self.angular_velocity)
        period_nanoseconds = int((2*math.pi/self.angular_velocity - period_seconds)*10**9)
        self.cycle_period = Duration(seconds = period_seconds,
                                            nanoseconds = period_nanoseconds)

        self.end_effector_position = Vector3()


        self.wait_for_solver()
        self.publisher_ = self.create_publisher(Vector3, 'end_effector_desired_trajectory', 10)
        self.timer_period = 0.01  # seconds
        self.timer = self.create_timer(self.timer_period, self.trajectory_callback)

    def trajectory_callback(self):
        self.time_now = self.clock.now()
        cycle_duration = (self.time_now - self.time_prev)
        cycle_duration_seconds= cycle_duration.nanoseconds/10**9 
        self.end_effector_position.x = 0.0
        self.end_effector_position.y = self.y + self.radius*math.sin(self.angular_velocity*cycle_duration_seconds)
        self.end_effector_position.z = 0.0
        if(cycle_duration > self.cycle_period):
            self.time_prev = self.time_now
            self.logger.info("Cycle ended!")
        self.publisher_.publish(self.end_effector_position)

    def wait_for_solver(self): # Czeka chwilę, aż noga ustawi się w pozycji startowej
        wait_time = Duration(seconds = 1)
        while(wait_time > (self.clock.now() - self.time_prev)):
            self.clock.sleep_for(Duration(nanoseconds = 10**8))
        self.logger.info("Trajectory generation has started!")
        self.time_prev = self.clock.now()

def main(args=None):
    rclpy.init(args=args)
    node = Linear_Trajectory("linear_trajectory")
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()