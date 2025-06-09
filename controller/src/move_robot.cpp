#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

class MoveRobot : public rclcpp::Node {
public:
    MoveRobot()
    : Node("move_robot")
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&MoveRobot::publish_velocity, this)
        );
        RCLCPP_INFO(this->get_logger(), "Initializing move_robot node...");
    }

private:
    void publish_velocity()
    {
        auto msg = geometry_msgs::msg::Twist();
        msg.linear.x = 0.2;
        msg.angular.z = 0.5;

        publisher_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Publishing: linear.x=%.2f, angular.z=%.2f", msg.linear.x, msg.angular.z);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MoveRobot>());
    rclcpp::shutdown();
    return 0;
}
