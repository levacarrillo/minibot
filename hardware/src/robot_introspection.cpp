#include "rclcpp/rclcpp.hpp"
#include "interfaces/msg/robot_status.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"


using std::placeholders::_1;
using namespace std::chrono_literals;

class RobotIntrospection : public rclcpp::Node {

public:
  RobotIntrospection() : Node ("robot_introspection") {
    RCLCPP_INFO(this->get_logger(), "INITIALIZING robot_introspection NODE BY LUIS GONZALEZ...");

    this->declare_parameter<int>("robot_id", this->robot_id);
    this->declare_parameter<std::string>("robot_name", this->robot_name);
    this->declare_parameter<std::string>("battery_supply_status", this->battery_supply_status);

    this->robot_id = this->get_parameter("robot_id").as_int();
    this->robot_name  = this->get_parameter("robot_name").as_string();
    this->battery_supply_status = this->get_parameter("battery_supply_status").as_string();
    // RCLCPP_INFO(this->get_logger(), "PARAMETERS LOADED robot_id=%d, name=%s, battery_capacity=%.2fV, battery_cells_number=%d, battery_supply_status=%s", robot_id, robot_name.c_str(), battery_capacity, battery_cells_number, battery_supply_status.c_str());
 
    subscription_=this->create_subscription<std_msgs::msg::Int32MultiArray>(
		    "sensors",
		    10,
		    std::bind(&RobotIntrospection::state_callback, this, _1));
    publisher_= this->create_publisher<interfaces::msg::RobotStatus>("robot_status", 10);

    timer_= this->create_wall_timer(
		    100ms, std::bind(&RobotIntrospection::timer_loop, this));
  }	

private:
  int robot_id;
  std::string robot_name;
  float max_battery_voltage = 8.4; // Volts
  float raspberrypi_temperature;
  float microcontroller_temperature;
  std::vector<float> battery_voltage = {0.0, 0.0};
  std::vector<int> battery_charge_percentage = {0, 0};
  std::string battery_supply_status;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr timer_once_;
  rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr subscription_;
  rclcpp::Publisher<interfaces::msg::RobotStatus>::SharedPtr publisher_;

  void state_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
    float min_perc = 0;//%
    float max_perc = 100;//%
    float min_analog = 2560;
    float max_analog = 4095;
    
    float battery1_percentage = min_perc + (msg->data[19] - min_analog) * (max_perc - min_perc) / (max_analog - min_analog);
    float battery2_percentage = min_perc + (msg->data[20] - min_analog) * (max_perc - min_perc) / (max_analog - min_analog);

    if (battery1_percentage < 0) {
        battery1_percentage = 0; 
    } else if (battery1_percentage > 100) {
	battery1_percentage = 100;
    }

    if (battery2_percentage < 0) {
        battery2_percentage = 0;
    } else if (battery2_percentage > 100) {
        battery2_percentage = 100;
    }

    // RCLCPP_INFO(this->get_logger(), "PERCENTAGE->%f", percentage);
    battery_voltage = {float(max_battery_voltage * battery1_percentage / 100), float(max_battery_voltage * battery2_percentage) };
    battery_charge_percentage = { int(battery1_percentage), int(battery2_percentage)};

    microcontroller_temperature = float(msg->data[22]);
  }
  
  void timer_loop() {
    
    FILE* pipe = popen("vcgencmd measure_temp", "r");
    if (!pipe) {
        RCLCPP_ERROR(this->get_logger(), "ERROR EXECUTING vcgencmd");
	return;
    }
    char buffer[128];
    std::string temp_result;

    while (fgets(buffer, sizeof(buffer), pipe) != nullptr) {
      temp_result += buffer;
    }
    pclose(pipe);

    // RCLCPP_INFO(this->get_logger(), "CPU TEMPERATURE-> %s", temp_result.c_str());
    temp_result.erase(temp_result.length() -2, 2);
    temp_result.erase(0, 5);

    raspberrypi_temperature = std::stof(temp_result);

    auto message = interfaces::msg::RobotStatus();
    message.robot_id    = robot_id;
    message.robot_name  = robot_name; 
    message.raspberrypi_temperature = raspberrypi_temperature;
    message.microcontroller_temperature = microcontroller_temperature;
    
    message.battery_voltage = battery_voltage;
    message.battery_charge_percentage = battery_charge_percentage;
    message.battery_supply_status = battery_supply_status;

    publisher_->publish(message);
  }
};


int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RobotIntrospection>());
  rclcpp::shutdown();
  return 0;
}
