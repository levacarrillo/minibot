#include "motion_planner/motion_planner.hpp"
#include "motion_planner/utilities/motion_planner_utilities.hpp"


MotionPlanner::MotionPlanner() : Node("motion_planner") {

  RCLCPP_INFO(this->get_logger(), "INITIALIZING MOTION_PLANNER NODE...");

  this->declare_parameter("behavior", this->selected_behavior);
  this->declare_parameter("run_behavior", this->behavior_running);
  this->declare_parameter("behavior_list", get_behavior_list());
  this->declare_parameter("max_advance", this->movement_params.max_advance);
  this->declare_parameter("max_turn_angle", this->movement_params.max_turn_angle);
  this->declare_parameter("light_threshold", this->light_sensors_data.light_threshold);
  
  this->light_readings_client = this->create_client<GetLightReadings>("get_light_readings");
  this->go_to_pose_client = rclcpp_action::create_client<GoToPose>(this, "go_to_pose");

  timer_ = this->create_wall_timer(
    1000ms, std::bind(&MotionPlanner::timer_callback, this));
}

void MotionPlanner::timer_callback() {
  this->selected_behavior = this->get_parameter("behavior").as_string();
  this->behavior_running  = this->get_parameter("run_behavior").as_bool();
  this->movement_params.max_advance = this->get_parameter("max_advance").as_double();
  this->movement_params.max_turn_angle = this->get_parameter("max_turn_angle").as_double();
  this->light_sensors_data.light_threshold = this->get_parameter("light_threshold").as_double();
}

bool MotionPlanner::behavior_is_running() {
  return this->behavior_running;
}

void MotionPlanner::stop_behavior() {
  this->behavior_running = false;
  this->set_parameter(rclcpp::Parameter("run_behavior", this->behavior_running));
  this->move_robot(this->stop);
}

void MotionPlanner::print_selected_behavior() {
  if (this->selected_behavior == "")
    RCLCPP_WARN(this->get_logger(), "NO BEHAVIOR SELECTED");
  else
    RCLCPP_INFO(this->get_logger(), "BEHAVIOR SELECTED->%s", this->selected_behavior.c_str());
}

Behaviors MotionPlanner::get_selected_behavior() {
  return find_behavior(this->selected_behavior);
}

MovementParams MotionPlanner::get_movement_params() {
  return this->movement_params;
}

LightSensorsData MotionPlanner::get_light_sensors_data() {
  while(!this->light_readings_client->wait_for_service(1s)) {
    RCLCPP_WARN(this->get_logger(), "SERVICE /get_light_readings NOT AVAILABLE, WAITING AGAIN...");
  }

  auto request = std::make_shared<GetLightReadings::Request>();
  auto result = this->light_readings_client->async_send_request(request);
  
  if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) == rclcpp::FutureReturnCode::SUCCESS) {
    auto response = result.get();
    
    this->light_sensors_data.light_readings      = response->readings;
    this->light_sensors_data.light_sensor_max_id = response->max_index;
    this->light_sensors_data.light_sensor_max    = response->max_value;
    this->light_sensors_data.light_direction = get_light_direction(response->readings);

  } else {
    RCLCPP_ERROR(this->get_logger(), "FAILED TO CALL SERVICE /get_light_readings");
  }

  return this->light_sensors_data;
}

void MotionPlanner::move_robot(movement_ movement) {
  while(!this->go_to_pose_client->wait_for_action_server(1s)) {
    RCLCPP_WARN(this->get_logger(), "ACTION SERVER /go_to_pose NOT AVAILABLE AFTER WAITING, RETRYING...");
  }
  auto goal_msg = GoToPose::Goal();
  goal_msg.angle = movement.twist;
  goal_msg.distance = movement.advance;
  
  auto goal_handle_future = this->go_to_pose_client->async_send_goal(goal_msg);

  if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), goal_handle_future) != rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(this->get_logger(), "FAILED TO SEND GOAL");
    return;
  }

  auto goal_handle = goal_handle_future.get();
  if (!goal_handle) {
    RCLCPP_ERROR(this->get_logger(), "GOAL REJECTED BY SERVER");
    return;
  }

  RCLCPP_INFO(this->get_logger(), "GOAL ACCEPTED, WAITING FOR RESULT...");

  auto result_future = this->go_to_pose_client->async_get_result(goal_handle);
  if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) != rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(this->get_logger(), "GOAL RESULT COULDN'T BEEN GOTTEN");
    return;
  }

  auto result = result_future.get();
  if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
    RCLCPP_INFO(this->get_logger(), "GOAL FINISHED SUCCESSFULLY");
  } else {
    RCLCPP_ERROR(this->get_logger(), "GOAL FINISHED WITH AN ERROR");
  }
}