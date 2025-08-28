#include "rclcpp/rclcpp.hpp"


class RobotIntrospection : public rclcpp::Node {
public:
  RobotIntrospection() : Node ("robot_introspection") {
    RCLCPP_INFO(this->get_logger(), "INITIALIZING ROBOT_INTROSPECTION NODE BY LUIS GONZALEZ...");
  }	
private:
  int robot_id;
  std::string robot_name;
  int temperature;
  float battery_voltage;
  float battery_capacity;
  int battery_cells_number;
  int battery_charge_percentage;

  std::string battery_supply_status;

};


int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RobotIntrospection>());
  rclcpp::shutdown();
  return 0;
}
