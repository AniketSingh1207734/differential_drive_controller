import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math
import matplotlib.pyplot as plt

class WaypointNavigator(Node):
    def __init__(self):
        super().__init__('waypoint_navigator')

        # Declare and initialize ROS 2 parameters
        self.declare_parameter('waypoint_1_x', 2.0)
        self.declare_parameter('waypoint_1_y', 1.0)
        self.declare_parameter('waypoint_2_x', 4.0)
        self.declare_parameter('waypoint_2_y', 3.0)
        self.declare_parameter('kp', 1.0)
        self.declare_parameter('ki', 0.0)
        self.declare_parameter('kd', 0.0)

        # Subscribe to odometry and laser scan data
        self.odom_sub_ = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.laser_sub_ = self.create_subscription(LaserScan, '/scan', self.laser_callback, 10)

        # Publisher for velocity commands
        self.cmd_vel_pub_ = self.create_publisher(Twist, '/cmd_vel', 10)

        # Waypoints and control parameters
        self.waypoints = [
            (self.get_parameter('waypoint_1_x').value, self.get_parameter('waypoint_1_y').value),
            (self.get_parameter('waypoint_2_x').value, self.get_parameter('waypoint_2_y').value)
        ]
        self.current_waypoint_index = 0

        # PID control parameters
        self.kp = self.get_parameter('kp').value
        self.ki = self.get_parameter('ki').value
        self.kd = self.get_parameter('kd').value

        # PID variables
        self.integral_error = 0.0
        self.previous_error = 0.0

        # Obstacle avoidance parameters
        self.obstacle_detected = False
        self.obstacle_distance_threshold = 0.5  # meters

        # Data for plotting the velocity over time
        self.time_data = []
        self.velocity_data = []
        self.start_time = self.get_clock().now().seconds_nanoseconds()[0]

    def odom_callback(self, msg):
        # Get robot's current position from odometry
        current_x = msg.pose.pose.position.x
        current_y = msg.pose.pose.position.y

        # Get current waypoint
        waypoint_x, waypoint_y = self.waypoints[self.current_waypoint_index]

        # Calculate the distance to the waypoint
        distance_error = math.sqrt((waypoint_x - current_x)**2 + (waypoint_y - current_y)**2)

        # Calculate the angle to the waypoint
        goal_angle = math.atan2(waypoint_y - current_y, waypoint_x - current_x)
        current_orientation = msg.pose.pose.orientation
        current_yaw = self.get_yaw_from_orientation(current_orientation)

        angle_error = goal_angle - current_yaw
        angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))  # Normalize angle

        # PID control for linear and angular velocity
        proportional = self.kp * distance_error
        self.integral_error += distance_error
        integral = self.ki * self.integral_error
        derivative = self.kd * (distance_error - self.previous_error)
        self.previous_error = distance_error

        control_signal_linear = proportional + integral + derivative
        control_signal_angular = self.kp * angle_error

        # Publish velocity commands
        velocity_msg = Twist()
        velocity_msg.linear.x = min(control_signal_linear, 0.5)  # Limit linear speed
        velocity_msg.angular.z = max(min(control_signal_angular, 0.5), -0.5)  # Limit angular speed
        self.cmd_vel_pub_.publish(velocity_msg)

        # Log velocity for plotting
        current_time = self.get_clock().now().seconds_nanoseconds()[0] - self.start_time
        self.time_data.append(current_time)
        self.velocity_data.append(velocity_msg.linear.x)

        # Check if waypoint is reached
        if distance_error < 0.1:
            self.current_waypoint_index += 1
            if self.current_waypoint_index >= len(self.waypoints):
                # Stop the robot
                velocity_msg.linear.x = 0.0
                velocity_msg.angular.z = 0.0
                self.cmd_vel_pub_.publish(velocity_msg)

                # Plot the velocity response
                self.plot_velocity_response()

                self.get_logger().info('All waypoints reached')
                rclpy.shutdown()

    def laser_callback(self, msg):
        # Check for obstacles within the distance threshold
        min_distance = min(msg.ranges)
        if min_distance < self.obstacle_distance_threshold:
            self.obstacle_detected = True
            self.get_logger().warn(f'Obstacle detected at {min_distance} meters')

            # Stop the robot when obstacle is detected
            velocity_msg = Twist()
            velocity_msg.linear.x = 0.0
            velocity_msg.angular.z = 0.0
            self.cmd_vel_pub_.publish(velocity_msg)

    def plot_velocity_response(self):
        """Plot the linear velocity over time."""
        plt.plot(self.time_data, self.velocity_data, label='Linear Velocity')
        plt.title('Velocity Over Time')
        plt.xlabel('Time (seconds)')
        plt.ylabel('Velocity (m/s)')
        plt.legend()
        plt.grid()
        plt.show()

    def get_yaw_from_orientation(self, orientation):
        """Convert quaternion to yaw angle (Z-axis rotation)."""
        siny_cosp = 2.0 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z)
        return math.atan2(siny_cosp, cosy_cosp)

def main(args=None):
    rclpy.init(args=args)
    navigator = WaypointNavigator()
    rclpy.spin(navigator)

if __name__ == '__main__':
    main()
