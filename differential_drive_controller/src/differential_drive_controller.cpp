#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "geometry_msgs/msg/twist.hpp"

class DifferentialDriveController : public rclcpp::Node
{
public:
    DifferentialDriveController() : Node("differential_drive_controller")
    {
        this->declare_parameter("wheelbase", 0.5);
        this->declare_parameter("wheel_radius", 0.1);
        this->declare_parameter("max_rpm", 100.0);

        // Subscriber and publishers
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10, std::bind(&DifferentialDriveController::cmdVelCallback, this, std::placeholders::_1));
        left_wheel_rpm_pub_ = this->create_publisher<std_msgs::msg::Float64>("/left_wheel_rpm", 10);
        right_wheel_rpm_pub_ = this->create_publisher<std_msgs::msg::Float64>("/right_wheel_rpm", 10);
    }

private:
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        double wheelbase = this->get_parameter("wheelbase").as_double();
        double wheel_radius = this->get_parameter("wheel_radius").as_double();
        double max_rpm = this->get_parameter("max_rpm").as_double();

        double linear_velocity = msg->linear.x;
        double angular_velocity = msg->angular.z;

        // Compute wheel velocities
        double v_left = linear_velocity - angular_velocity * (wheelbase / 2);
        double v_right = linear_velocity + angular_velocity * (wheelbase / 2);

        // Convert to RPM
        double left_rpm = (v_left / (2 * M_PI * wheel_radius)) * 60.0;
        double right_rpm = (v_right / (2 * M_PI * wheel_radius)) * 60.0;

        // Clamp to max RPM
        left_rpm = std::min(max_rpm, std::max(-max_rpm, left_rpm));
        right_rpm = std::min(max_rpm, std::max(-max_rpm, right_rpm));

        // Publish RPMs
        std_msgs::msg::Float64 left_rpm_msg;
        std_msgs::msg::Float64 right_rpm_msg;
        left_rpm_msg.data = left_rpm;
        right_rpm_msg.data = right_rpm;
        left_wheel_rpm_pub_->publish(left_rpm_msg);
        right_wheel_rpm_pub_->publish(right_rpm_msg);
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr left_wheel_rpm_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr right_wheel_rpm_pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DifferentialDriveController>());
    rclcpp::shutdown();
    return 0;
}
