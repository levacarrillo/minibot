#ifndef MOTION_PLANNER__MOTION_PLANNER_HPP_
#define MOTION_PLANNER__MOTION_PLANNER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "interfaces/action/go_to_pose.hpp"
#include "interfaces/srv/get_light_readings.hpp"
#include "motion_planner/utilities/structures.hpp"

using namespace std::placeholders;
using namespace std::chrono_literals;

class MotionPlanner : public rclcpp::Node {
  public:
    using GoToPose = interfaces::action::GoToPose;
    using GoalHandleGoToPose = rclcpp_action::ClientGoalHandle<GoToPose>;
    using GetLightReadings = interfaces::srv::GetLightReadings;

    MotionPlanner();
    void stop_behavior();
    bool behavior_is_running();
    void move_robot(movement_);
    void print_selected_behavior();
    
    Behaviors get_selected_behavior();
    MovementParams get_movement_params();
    LightSensorsData get_light_sensors_data();

  private:
    movement_ stop;
    bool behavior_running;
    std::string selected_behavior;    
    MovementParams movement_params;
    LightSensorsData light_sensors_data;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Client<GetLightReadings>::SharedPtr light_readings_client;
    rclcpp_action::Client<GoToPose>::SharedPtr go_to_pose_client;
    
    void timer_callback();
};

#endif
