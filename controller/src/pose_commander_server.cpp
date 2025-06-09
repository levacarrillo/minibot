#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "interfaces/srv/go_to_pose.hpp"

class PoseCommanderServer : public rclcpp::Node {
public:
    PoseCommanderServer() : Node("pose_commander_server") {
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        srv_ = this->create_service<interfaces::srv::GoToPose>(
            "/go_to_pose",
            std::bind(&PoseCommanderServer::handle_request, this, std::placeholders::_1, std::placeholders::_2)
        );
        RCLCPP_INFO(this->get_logger(), "Initializing pose_commander_server node...");
    }

private:
    void handle_request(
        const std::shared_ptr<interfaces::srv::GoToPose::Request> request,
        std::shared_ptr<interfaces::srv::GoToPose::Response> response
    ) {
        double angle = request->angle;
        double distance = request->distance;

        RCLCPP_INFO(this->get_logger(), "Recieved: %.2f rad, %.2f m", angle, distance);

        geometry_msgs::msg::Twist cmd_vel_msg;

        cmd_vel_msg.angular.z = (angle > 0 ? 0.5 : -0.5);
        cmd_vel_pub_->publish(cmd_vel_msg);
        rclcpp::sleep_for(std::chrono::milliseconds(int(1000 * std::abs(angle) / 0.5)));

        cmd_vel_msg.angular.z = 0.0;
        cmd_vel_msg.linear.x = (distance > 0 ? 0.2 : -0.2);
        cmd_vel_pub_->publish(cmd_vel_msg);
        rclcpp::sleep_for(std::chrono::milliseconds(int(1000 * std::abs(distance) / 0.2)));

        cmd_vel_msg.linear.x = 0.0;
        cmd_vel_pub_->publish(cmd_vel_msg);

        response->success = true;
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Service<interfaces::srv::GoToPose>::SharedPtr srv_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<PoseCommanderServer>();
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}