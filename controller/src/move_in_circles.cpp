#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

class MoveInCircles : public rclcpp::Node {
public:
    MoveInCircles()
    : Node("move_in_circles")
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&MoveInCircles::publish_velocity, this)
        );
        RCLCPP_INFO(this->get_logger(), "Initializing move_in_circles node...");
    }

private:
    void publish_velocity()
    {
        auto msg = geometry_msgs::msg::Twist();
        msg.linear.x = 0.2;
        msg.angular.z = 0.5;

        publisher_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "PUBLISHING: linear.x=%.2f, angular.z=%.2f", msg.linear.x, msg.angular.z);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MoveInCircles>());
    rclcpp::shutdown();
    return 0;
}
