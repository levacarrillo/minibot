#ifndef MOTION_PLANNER__MOTION_PLANNER_HPP_
#define MOTION_PLANNER__MOTION_PLANNER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "interfaces/srv/get_scan.hpp"
#include "interfaces/srv/get_params.hpp"
#include "interfaces/srv/set_params.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "interfaces/action/go_to_pose.hpp"
#include "interfaces/srv/get_light_readings.hpp"
#include "motion_planner/utilities/structures.hpp"

using namespace std::placeholders;
using namespace std::chrono_literals;

class MotionPlanner : public rclcpp::Node {
  public:
    using GetScan  = interfaces::srv::GetScan;
    using GetParams = interfaces::srv::GetParams;
    using SetParams = interfaces::srv::SetParams;
    using GoToPose = interfaces::action::GoToPose;
    using GetLightReadings   = interfaces::srv::GetLightReadings;
    using GoalHandleGoToPose = rclcpp_action::ClientGoalHandle<GoToPose>;

    MotionPlanner();
    void stop_behavior();
    bool steps_exceeded();
    int get_current_step();
    void set_next_state(int);
    void move_robot(Movement);
    bool behavior_is_running();
    void print_selected_behavior();
    
    Sensors get_sensors_data();
    Behaviors get_selected_behavior();
    MovementParams get_movement_params();
    
  private:
    Movement stop;
    bool behavior_running;
    Sensors sensors_data;
    std::string selected_behavior;    
    MovementParams movement_params;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Service<GetParams>::SharedPtr get_params_server;
    rclcpp::Service<SetParams>::SharedPtr set_params_server;
    rclcpp::Client<GetScan>::SharedPtr  laser_readings_client;
    rclcpp_action::Client<GoToPose>::SharedPtr go_to_pose_client;
    rclcpp::Client<GetLightReadings>::SharedPtr light_readings_client;
    void get_params(const std::shared_ptr<GetParams::Request>, std::shared_ptr<GetParams::Response>);
    void set_params(const std::shared_ptr<SetParams::Request>, std::shared_ptr<SetParams::Response>);
    
    void increase_steps();
    void timer_callback();
};

#endif
