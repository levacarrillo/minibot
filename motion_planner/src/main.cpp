#include "motion_planner/motion_planner.hpp"
#include "motion_planner/utilities/utilities.hpp"
#include "motion_planner/state_machines/light_follower.hpp"
#include "motion_planner/state_machines/sm_destination.hpp"
#include "motion_planner/state_machines/sm_avoid_obstacles.hpp"
#include "motion_planner/state_machines/sm_avoidance_destination.hpp"
#include "motion_planner/state_machines/user_sm.hpp"


int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MotionPlanner>();

  
  while (rclcpp::ok()) {
    
    bool behavior_runnig = node->behavior_is_running();

    if(behavior_runnig) {
      Movement movement;
      RCLCPP_INFO(node->get_logger(), "\n \n  MOTION PLANNER \n____________________________\n");
      node->print_selected_behavior();
      if(node->steps_exceeded()) continue;
      Behaviors behavior = node->get_selected_behavior();
      MovementParams movement_params = node->get_movement_params();
      Sensors sensors_data  = node->get_sensors_data();


      bool valid_behavior = false;

      switch(behavior) {
        case NO_SELECTED:
          valid_behavior = false;
        break;
        
        case LIGHT_FOLLOWER:
          behavior_runnig = light_follower(sensors_data, movement_params, &movement);
          valid_behavior = true;
        break;
        
        case SM_DESTINATION:
          behavior_runnig = sm_destination(sensors_data, &movement_params, &movement);
          valid_behavior = true;
        break;
        
        case SM_AVOID_OBSTACLES:
          sm_avoid_obstacles(sensors_data, &movement_params, &movement);
          valid_behavior = true;
        break;
        
        case SM_AVOIDANCE_DESTINATION:
          behavior_runnig = sm_avoidance_destination(sensors_data, &movement_params, &movement);
          valid_behavior = true;
        break;

        case USER_SM:
          behavior_runnig = user_sm(sensors_data, &movement_params, &movement);
          valid_behavior = true;
        break;
        
        default:
          RCLCPP_ERROR(node->get_logger(), "BEHAVIOR NO RECOGNIZED");
          valid_behavior = false;
        break;
      }

      if (behavior_runnig && valid_behavior) {
        node->set_next_state(movement_params.state);
        RCLCPP_INFO(node->get_logger(), "CURRENT STEP->%d", node->get_current_step());  
        RCLCPP_INFO(node->get_logger(), "MOVEMENT: TWIST->%.2f, ADVANCE->%.2f", movement.twist, movement.advance);
        node->move_robot(movement);
      } else {
        RCLCPP_WARN(node->get_logger(), "\n \n  BEHAVIOR STOPPED  \n____________________________\n");
        node->stop_behavior();
      }
    }

    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  RCLCPP_INFO(node->get_logger(), "INTERRUPTED MOTION_PLANNER NODE. EXITING.");

  rclcpp::shutdown();
  return 0;
}
