#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float64.hpp"

class DifferentialDriveController : public rclcpp::Node {
public:
    DifferentialDriveController() : Node("differential_drive_controller") {
        // Declare ROS parameters with default values
        this->declare_parameter<double>("wheelbase", 0.300); // 30 cm
        this->declare_parameter<double>("wheel_radius", 0.033); // 3.3 cm
        this->declare_parameter<double>("max_rpm", 150.0); 

        // Get parameter values
        wheelbase_ = this->get_parameter("wheelbase").as_double();
        wheel_radius_ = this->get_parameter("wheel_radius").as_double();
        max_rpm_ = this->get_parameter("max_rpm").as_double();

        // Subscribe to velocity command
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10, std::bind(&DifferentialDriveController::cmdVelCallback, this, std::placeholders::_1));

        // Publishers for left and right wheel RPM
        left_wheel_rpm_pub_ = this->create_publisher<std_msgs::msg::Float64>("/left_wheel_rpm", 10);
        right_wheel_rpm_pub_ = this->create_publisher<std_msgs::msg::Float64>("/right_wheel_rpm", 10);
    }

private:
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        // Extract linear and angular velocities from the Twist message
        double linear_velocity = msg->linear.x;
        double angular_velocity = msg->angular.z;

        // Compute the linear velocity for each wheel
        double v_left = linear_velocity - (angular_velocity * wheelbase_ / 2.0);
        double v_right = linear_velocity + (angular_velocity * wheelbase_ / 2.0);

        // Convert linear velocities to RPM
        double left_wheel_rpm = (v_left / (2.0 * M_PI * wheel_radius_)) * 60.0;
        double right_wheel_rpm = (v_right / (2.0 * M_PI * wheel_radius_)) * 60.0;

        // Ensure RPMs do not exceed max RPM
        left_wheel_rpm = std::min(left_wheel_rpm, max_rpm_);
        right_wheel_rpm = std::min(right_wheel_rpm, max_rpm_);

        // Publish RPM values
        auto left_rpm_msg = std_msgs::msg::Float64();
        left_rpm_msg.data = left_wheel_rpm;
        left_wheel_rpm_pub_->publish(left_rpm_msg);

        auto right_rpm_msg = std_msgs::msg::Float64();
        right_rpm_msg.data = right_wheel_rpm;
        right_wheel_rpm_pub_->publish(right_rpm_msg);
    }

    // ROS parameters
    double wheelbase_;
    double wheel_radius_;
    double max_rpm_;

    // ROS publishers and subscribers
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr left_wheel_rpm_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr right_wheel_rpm_pub_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DifferentialDriveController>());
    rclcpp::shutdown();
    return 0;
}
