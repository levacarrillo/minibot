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
      movement_ movement;
      RCLCPP_INFO(node->get_logger(), "\n \n  MOTION PLANNER \n____________________________\n");
      node->print_selected_behavior();
      Behaviors behavior = node->get_selected_behavior();
      MovementParams movement_params = node->get_movement_params();
      LightSensorsData light_data = node->get_light_sensors_data();
      
      bool valid_behavior = false;

      switch(behavior) {
        case NONE:
          valid_behavior = false;
        break;
        
        case LIGHT_FOLLOWER:
          behavior_runnig = light_follower(light_data, movement_params, &movement);
          valid_behavior = true;
        break;
        
        case SM_DESTINATION:
          behavior_runnig = sm_destination(light_data, &movement_params, &movement);
          valid_behavior = true;
        break;
        
        case SM_AVOID_OBSTACLES:
          behavior_runnig = sm_avoid_obstacles(light_data, movement_params, &movement);
          valid_behavior = true;
        break;
        
        case SM_AVOIDANCE_DESTINATION:
          behavior_runnig = sm_avoidance_destination(light_data, movement_params, &movement);
          valid_behavior = true;
        break;

        case USER_SM:
          behavior_runnig = user_sm(light_data, movement_params, &movement);
          valid_behavior = true;
        break;
        
        default:
          RCLCPP_ERROR(node->get_logger(), "************* BEHAVIOR NO RECOGNIZED *************");
          valid_behavior = false;
        break;
      }

      if (behavior_runnig && valid_behavior) {
        // RCLCPP_INFO(node->get_logger(), "MOVEMENT: TWIST->%.2f ADVANCE->%.2f", movement.twist, movement.advance);
        node->move_robot(movement);
      } else {
        RCLCPP_INFO(node->get_logger(), "\n \n  STOPPED BEHAVIOR  \n____________________________\n");
        node->stop_behavior();
      }
    }

    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }

  RCLCPP_INFO(node->get_logger(), "INTERRUPTED MOTION_PLANNER NODE. EXITING.");

  rclcpp::shutdown();
  return 0;
}
