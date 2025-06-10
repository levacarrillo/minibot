#ifndef MOTION_PLANNER__MOTION_PLANNER_HPP_
#define MOTION_PLANNER__MOTION_PLANNER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "interfaces/srv/get_scan.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "interfaces/action/go_to_pose.hpp"
#include "interfaces/srv/get_light_readings.hpp"
#include "motion_planner/utilities/structures.hpp"

using namespace std::placeholders;
using namespace std::chrono_literals;

class MotionPlanner : public rclcpp::Node {
  public:
    using GetScan = interfaces::srv::GetScan;
    using GoToPose = interfaces::action::GoToPose;
    using GoalHandleGoToPose = rclcpp_action::ClientGoalHandle<GoToPose>;
    using GetLightReadings = interfaces::srv::GetLightReadings;

    MotionPlanner();
    void stop_behavior();
    bool steps_exceeded();
    int get_current_step();
    void set_next_state(int);
    void move_robot(Movement);
    bool behavior_is_running();
    void print_selected_behavior();
    
    Behaviors get_selected_behavior();
    MovementParams get_movement_params();
    LightSensorsData get_light_sensors_data();
    LaserSensorData  get_laser_sensor_data();
    
    private:
    Movement stop;
    bool behavior_running;
    std::string selected_behavior;    
    MovementParams movement_params;
    LightSensorsData light_sensors_data;
    LaserSensorData laser_sensor_data;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Client<GetScan>::SharedPtr laser_readings_client;
    rclcpp_action::Client<GoToPose>::SharedPtr go_to_pose_client;
    rclcpp::Client<GetLightReadings>::SharedPtr light_readings_client;
    
    void increase_steps();
    void timer_callback();
};

#endif
