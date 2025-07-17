#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "interfaces/srv/get_vel_params.hpp"
#include "interfaces/srv/set_vel_params.hpp"
#include "interfaces/action/go_to_pose.hpp"
#include "geometry_msgs/msg/twist.hpp"


using namespace std::placeholders;
using GoToPose = interfaces::action::GoToPose;
using GetVelParams = interfaces::srv::GetVelParams;
using SetVelParams = interfaces::srv::SetVelParams;
using GoalHandleGoToPose = rclcpp_action::ServerGoalHandle<GoToPose>;

class PoseCommander : public rclcpp::Node {
public:
    PoseCommander() : Node("pose_commander") {
        RCLCPP_INFO(this->get_logger(), "INTITIALITZING POSE_COMMANDER NODE...");

        this->declare_parameter("linear_velocity", this->linear_velocity);
        this->declare_parameter("angular_velocity", this->angular_velocity);
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        set_vel_service = this->create_service<SetVelParams>(
            "set_vel_params",
            std::bind(&PoseCommander::handler_set_velocity, this, _1, _2)
        );
        get_vel_service = this->create_service<GetVelParams>(
            "get_vel_params",
            std::bind(&PoseCommander::handler_get_velocity, this, _1, _2)
        );
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
    rclcpp_action::Server<GoToPose>::SharedPtr action_server_;
    rclcpp::Service<GetVelParams>::SharedPtr get_vel_service;
    rclcpp::Service<SetVelParams>::SharedPtr set_vel_service;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

    
    void handler_get_velocity(const std::shared_ptr<GetVelParams::Request> /*request*/,
        std::shared_ptr<GetVelParams::Response> response)
    {
        response->linear_velocity  = get_parameter("linear_velocity").as_double();
        response->angular_velocity = get_parameter("angular_velocity").as_double();
    }

    void handler_set_velocity(const std::shared_ptr<SetVelParams::Request> request,
        std::shared_ptr<SetVelParams::Response> response)
    {
        this->set_parameter(rclcpp::Parameter("linear_velocity",  request->linear_velocity));
        this->set_parameter(rclcpp::Parameter("angular_velocity", request->angular_velocity));
        response->success = true;
    }

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID &/*uuid*/,
        std::shared_ptr<const GoToPose::Goal> /* goal */) {
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

        RCLCPP_INFO(this->get_logger(), "MOVING WITH: linear_velocity->%f and angular_velocity->%f", 
                                                        this->linear_velocity, this->angular_velocity);
        auto feedback = std::make_shared<GoToPose::Feedback>();
        auto result = std::make_shared<GoToPose::Result>();

        const auto goal = goal_handle->get_goal();
        geometry_msgs::msg::Twist cmd;


        feedback->feedback = "Twisting...";
        goal_handle->publish_feedback(feedback);
        cmd.angular.z = (goal->angle > 0 ? this->angular_velocity : - this->angular_velocity);
        cmd_vel_pub_->publish(cmd);

        double time_twist = std::abs(goal->angle) / 0.5;
        rclcpp::Time start_twist = this->now();
        rclcpp::Duration duration_twist = rclcpp::Duration::from_seconds(time_twist);

        while (this->now() - start_twist < duration_twist) {
            if (goal_handle->is_canceling()) {
                RCLCPP_INFO(this->get_logger(), "CANCELING MOVEMENT...");
                cmd.angular.z = 0.0;
                cmd_vel_pub_->publish(cmd);
                result->success = false;
                goal_handle->canceled(result);
                return;
            }
            rclcpp::sleep_for(std::chrono::milliseconds(100));
        }


        feedback->feedback = "Moving...";
        goal_handle->publish_feedback(feedback);
        cmd.angular.z = 0.0;
        cmd.linear.x = (goal->distance > 0 ? this->linear_velocity : - this->linear_velocity);
        cmd_vel_pub_->publish(cmd);

        double time_move = std::abs(goal->distance) / 0.2;
        rclcpp::Time start_move = this->now();
        rclcpp::Duration duration_move = rclcpp::Duration::from_seconds(time_move);

        while (this->now() - start_move < duration_move) {
            if (goal_handle->is_canceling()) {
                cmd.linear.x = 0.0;
                cmd_vel_pub_->publish(cmd);
                result->success = false;
                goal_handle->canceled(result);
                return;
            }
            rclcpp::sleep_for(std::chrono::milliseconds(100));
        }

        
        cmd.linear.x  = 0.0;
        cmd.angular.z = 0.0;
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