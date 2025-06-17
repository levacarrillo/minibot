#include "motion_planner/motion_planner.hpp"
#include "motion_planner/utilities/motion_planner_utilities.hpp"


MotionPlanner::MotionPlanner() : Node("motion_planner") {

  RCLCPP_INFO(this->get_logger(), "INITIALIZING MOTION_PLANNER NODE...");

  this->declare_parameter("behavior",     this->selected_behavior);
  this->declare_parameter("run_behavior", this->behavior_running);
  this->declare_parameter("behavior_list", get_behavior_list());
  this->declare_parameter("current_state",  this->movement_params.state);
  this->declare_parameter("step",           this->movement_params.step);
  this->declare_parameter("max_steps",      this->movement_params.max_steps);
  this->declare_parameter("max_advance",    this->movement_params.max_advance);
  this->declare_parameter("max_turn_angle", this->movement_params.max_turn_angle);
  this->declare_parameter("light_threshold", this->light_sensors_data.light_threshold);
  this->declare_parameter("laser_threshold", this->laser_sensor_data.laser_threshold);
  
  this->laser_readings_client = this->create_client<GetScan>("get_scan");
  this->get_params_server      = this->create_service<GetParams>("get_params",
                                std::bind(&MotionPlanner::get_params, this, _1, _2));
  this->set_params_server      = this->create_service<SetParams>("set_params",
                                std::bind(&MotionPlanner::set_params, this, _1, _2));
  this->go_to_pose_client     = rclcpp_action::create_client<GoToPose>(this, "go_to_pose");
  this->light_readings_client = this->create_client<GetLightReadings>("get_light_readings");

  timer_ = this->create_wall_timer(
    1000ms, std::bind(&MotionPlanner::timer_callback, this));
}

void MotionPlanner::timer_callback() {
  this->selected_behavior = this->get_parameter("behavior").as_string();
  this->behavior_running  = this->get_parameter("run_behavior").as_bool();
  this->movement_params.max_steps   = this->get_parameter("max_steps").as_int();
  this->movement_params.max_advance = this->get_parameter("max_advance").as_double();
  this->movement_params.max_turn_angle = this->get_parameter("max_turn_angle").as_double();
  this->light_sensors_data.light_threshold = this->get_parameter("light_threshold").as_double();
  this->laser_sensor_data.laser_threshold  = this->get_parameter("laser_threshold").as_double();

}

bool MotionPlanner::behavior_is_running() {
  return this->behavior_running;
}

void MotionPlanner::set_next_state(int state) {
  this->set_parameter(rclcpp::Parameter("current_state", state));
  this->movement_params.state =  state;
}

void MotionPlanner::stop_behavior() {
  this->behavior_running = false;
  this->set_parameter(rclcpp::Parameter("run_behavior", this->behavior_running));
  this->move_robot(this->stop);
}

int MotionPlanner::get_current_step() {
  return this->movement_params.step;
}

bool MotionPlanner::steps_exceeded() {
  if (this->movement_params.step >= this->movement_params.max_steps) {
    RCLCPP_WARN(this->get_logger(), "STEPS NUMBER HAS REACHED ITS MAXIMUN VALUE->%d",
                                    this->movement_params.max_steps);
    this->stop_behavior();
    return true;
  }
  return false;
}

void MotionPlanner::increase_steps() {
  this->movement_params.step++;
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

void MotionPlanner::get_params(const std::shared_ptr<GetParams::Request> /*request*/,
                                        std::shared_ptr<GetParams::Response> response)
                                        {
  response->behavior        = this->selected_behavior;
  response->run_behavior    = this->behavior_running;
  response->behavior_list   = get_behavior_list();
  response->current_state   = this->movement_params.state;
  response->step            = this->movement_params.step;
  response->max_steps       = this->movement_params.max_steps;
  response->max_advance     = this->movement_params.max_advance;
  response->max_turn_angle  = this->movement_params.max_turn_angle;
  response->light_threshold = this->light_sensors_data.light_threshold;
  response->laser_threshold = this->laser_sensor_data.laser_threshold;
}

void MotionPlanner::set_params(const std::shared_ptr<SetParams::Request> request,
                                        std::shared_ptr<SetParams::Response> response) {

  this->selected_behavior                  = request->behavior;
  this->behavior_running                   = request->run_behavior;
  this->movement_params.max_steps          = request->max_steps;
  this->movement_params.max_advance        = request->max_advance ;
  this->movement_params.max_turn_angle     = request->max_turn_angle;
  this->light_sensors_data.light_threshold = request->light_threshold;
  this->laser_sensor_data.laser_threshold  = request->laser_threshold;
  
  response->success = true;
}

LightSensorsData MotionPlanner::get_light_sensors_data() {
  while(!this->light_readings_client->wait_for_service(1s)) {
    RCLCPP_WARN(this->get_logger(), "SERVICE /get_light_readings NOT AVAILABLE, WAITING AGAIN...");
  }

  auto request = std::make_shared<GetLightReadings::Request>();
  auto result = this->light_readings_client->async_send_request(request);
  
  if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result)
                                                      == rclcpp::FutureReturnCode::SUCCESS) {
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

LaserSensorData MotionPlanner::get_laser_sensor_data() {
  while(!this->laser_readings_client->wait_for_service(1s)) {
    RCLCPP_WARN(this->get_logger(), "SERVICE /get_scan NOT AVAILABLE, WAITING AGAIN...");   
  }
  auto request = std::make_shared<GetScan::Request>();
  auto result = this->laser_readings_client->async_send_request(request);
  if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result)
                                                        == rclcpp::FutureReturnCode::SUCCESS) {
    auto response = result.get();
    this->laser_sensor_data.laser_readings = response->scan;
    this->laser_sensor_data.obstacle_direction = get_obstacle_direction(this->laser_sensor_data);
  } else {
    RCLCPP_ERROR(this->get_logger(), "FAILED TO CALL SERVICE /get_scan");
  }

  return this->laser_sensor_data;
}

void MotionPlanner::move_robot(Movement movement) {
  while(!this->go_to_pose_client->wait_for_action_server(1s)) {
    RCLCPP_WARN(this->get_logger(),
                            "ACTION SERVER /go_to_pose NOT AVAILABLE AFTER WAITING, RETRYING...");
  }
  auto goal_msg = GoToPose::Goal();
  goal_msg.angle = movement.twist;
  goal_msg.distance = movement.advance;
  
  auto goal_handle_future = this->go_to_pose_client->async_send_goal(goal_msg);

  if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), goal_handle_future)
                                                          != rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(this->get_logger(), "FAILED TO SEND MOVEMENT");
    return;
  }

  auto goal_handle = goal_handle_future.get();
  if (!goal_handle) {
    RCLCPP_ERROR(this->get_logger(), "MOVEMENT REJECTED BY SERVER");
    return;
  }

  // RCLCPP_INFO(this->get_logger(), "MOVEMENT ACCEPTED, WAITING FOR RESULT...");

  auto result_future = this->go_to_pose_client->async_get_result(goal_handle);
  if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future)
                                                          != rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(this->get_logger(), "MOVEMENT RESULT COULDN'T BE FOUND");
    return;
  }

  auto result = result_future.get();
  if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
    RCLCPP_INFO(this->get_logger(), "MOVEMENT FINISHED SUCCESSFULLY");
  } else {
    RCLCPP_ERROR(this->get_logger(), "MOVEMENT FINISHED WITH AN ERROR");
  }
  this->increase_steps();
}