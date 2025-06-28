import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
import math
from rclpy.duration import Duration

class Circle_Trajectory(Node):
    def __init__(self,name):
        super().__init__(name)
        self.INIT_PHASE = 0
        self.CIRCLE_PHASE = 1
        self.phase = self.INIT_PHASE
        self.declare_parameter("start_position", [0.0, -0.35])
        self.declare_parameter("radius",0.1)
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
        self.subscriber = self.create_subscription(Vector3, 'end_effector_actual_trajectory', self.phase_callback, 10)
        self.publisher_ = self.create_publisher(Vector3, 'end_effector_desired_trajectory', 10)
        self.timer_period = 0.001  # seconds
        self.timer = self.create_timer(self.timer_period, self.trajectory_callback)

    def trajectory_callback(self):
        if(self.phase == self.CIRCLE_PHASE):
            self.time_now = self.clock.now()
            cycle_duration = (self.time_now - self.time_prev)
            cycle_duration_seconds= cycle_duration.nanoseconds/10**9 
            self.end_effector_position.x = self.x + self.radius*math.cos(self.angular_velocity*cycle_duration_seconds)
            self.end_effector_position.y = self.y + self.radius*math.sin(self.angular_velocity*cycle_duration_seconds)
            self.end_effector_position.z = 0.0
            if(cycle_duration > self.cycle_period):
                self.time_prev = self.time_now
                self.logger.info("Cycle ended!")
            self.publisher_.publish(self.end_effector_position)
        

    def wait_for_solver(self): # Czeka chwilę, aż noga ustawi się w pozycji startowej
        wait_time = Duration(seconds = 2)
        while(wait_time > (self.clock.now() - self.time_prev)):
            self.clock.sleep_for(Duration(nanoseconds = 10**8))
        self.logger.info("Circle trajectory generation has started!")
        self.time_prev = self.clock.now()
    
    def phase_callback(self, msg):
        if(self.phase == self.INIT_PHASE):
            actual_position = [msg.x, msg.y]
            starting_position = [self.x + self.radius, self.y]
            self.logger.info("Movement to starting position!")
            while(not math.isclose(starting_position[0], actual_position[0],abs_tol = 0.05) 
                  and not math.isclose(starting_position[1], actual_position[1],abs_tol = 0.05)):
                error = [starting_position[0] - actual_position[0], 
                         starting_position[1] - actual_position[1]]
                self.end_effector_position.x = actual_position[0] + 0.01*error[0]
                self.end_effector_position.x = actual_position[1] + 0.01*error[1]
                self.publisher_.publish(self.end_effector_position)
            self.phase = self.CIRCLE_PHASE
            self.time_prev = self.clock.now()




def main(args=None):
    rclpy.init(args=args)
    node = Circle_Trajectory("circle_trajectory")
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()