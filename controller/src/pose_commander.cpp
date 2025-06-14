#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "interfaces/action/go_to_pose.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::placeholders;
using GoToPose = interfaces::action::GoToPose;
using GoalHandleGoToPose = rclcpp_action::ServerGoalHandle<GoToPose>;

class PoseCommander : public rclcpp::Node {
public:
    PoseCommander() : Node("pose_commander") {
        RCLCPP_INFO(this->get_logger(), "INTITIALITZING POSE_COMMANDER NODE...");

        this->declare_parameter("linear_velocity", this->linear_velocity);
        this->declare_parameter("angular_velocity", this->angular_velocity);
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        action_server_ = rclcpp_action::create_server<GoToPose>(
            this,
            "go_to_pose",
            std::bind(&PoseCommander::handle_goal, this, _1, _2),
            std::bind(&PoseCommander::handle_cancel, this, _1),
            std::bind(&PoseCommander::handle_accepted, this, _1)
        );
    }

private:
    float linear_velocity;
    float angular_velocity;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp_action::Server<GoToPose>::SharedPtr action_server_;

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID &/*uuid*/,
        std::shared_ptr<const GoToPose::Goal> goal) {
        // RCLCPP_INFO(this->get_logger(), "GOAL RECEIVED: TWIST->%.2f rad, ADVANCE->%.2f m", goal->angle, goal->distance);
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleGoToPose> goal_handle) {
        // RCLCPP_INFO(this->get_logger(), "GOAL ACCEPTED");
        std::thread{std::bind(&PoseCommander::execute, this, _1), goal_handle}.detach();
    }

    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleGoToPose> /*goal_handle*/) {
        RCLCPP_INFO(this->get_logger(), "CANCEL REQUESTED");
        return rclcpp_action::CancelResponse::ACCEPT;
    }


    void execute(const std::shared_ptr<GoalHandleGoToPose> goal_handle) {
        this->linear_velocity  = this->get_parameter("linear_velocity").as_double();
        this->angular_velocity = this->get_parameter("angular_velocity").as_double();

        auto feedback = std::make_shared<GoToPose::Feedback>();
        auto result = std::make_shared<GoToPose::Result>();

        const auto goal = goal_handle->get_goal();
        geometry_msgs::msg::Twist cmd;


        feedback->feedback = "Twisting...";
        goal_handle->publish_feedback(feedback);
        cmd.angular.z = (goal->angle > 0 ? this->angular_velocity : - this->angular_velocity);
        cmd_vel_pub_->publish(cmd);
        rclcpp::sleep_for(std::chrono::milliseconds(int(1000 * std::abs(goal->angle) / 0.5)));


        feedback->feedback = "Moving...";
        goal_handle->publish_feedback(feedback);
        cmd.angular.z = 0.0;
        cmd.linear.x = (goal->distance > 0 ? this->linear_velocity : - this->linear_velocity);
        cmd_vel_pub_->publish(cmd);
        rclcpp::sleep_for(std::chrono::milliseconds(int(1000 * std::abs(goal->distance) / 0.2)));

        
        cmd.linear.x = 0.0;
        cmd_vel_pub_->publish(cmd);

        result->success = true;
        goal_handle->succeed(result);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<PoseCommander>();
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}