#include "gpiod.hpp"
#include "rclcpp/rclcpp.hpp"
#include "interfaces/msg/robot_status.hpp"
#include "interfaces/srv/reset_micro.hpp"
#include "interfaces/srv/turn_on_fan.hpp"
#include "interfaces/srv/disconnect_source.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"


using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;

#define GPIO_DISCONNECT_SOURCE           2
#define GPIO_RESET_MICROCONTROLLER       3
#define GPIO_FAN_CONTROLLER              4
#define GPIO_CHECK_CHARGER_CONNECTION   14 
#define GPIO_BATTERY1_LOW_DETECTION     15
#define GPIO_BATTERY2_LOW_DETECTION     17
#define GPIO_BATTERY1_FULL_DETECTION    18
#define GPIO_BATTERY2_FULL_DETECTION    27


class RobotIntrospection : public rclcpp::Node {

public:
  RobotIntrospection() : Node ("robot_introspection") {
    RCLCPP_INFO(this->get_logger(), "INITIALIZING robot_introspection NODE BY LUIS GONZALEZ...");

    chip_ = gpiod::chip("gpiochip0");

    for (int pin : input_pins_) {
      auto line = chip_.get_line(pin);
      line.request({"robot_introspection",
      gpiod::line_request::DIRECTION_INPUT, 0});
      input_lines_.push_back(line);
    }

    for (int pin : output_pins_) {
      auto line = chip_.get_line(pin);
      line.request({"robot_introspection",
      gpiod::line_request::DIRECTION_OUTPUT, 0}, 0);
      output_lines_.push_back(line);
    }



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

    service_ = this->create_service<interfaces::srv::ResetMicro>("reset_microcontroller", std::bind(&RobotIntrospection::reset_micro, this, _1, _2));
    fan_service_ = this->create_service<interfaces::srv::TurnOnFan>("turn_on_fan", std::bind(&RobotIntrospection::turn_on_fan, this, _1, _2));
    source_service_ = this->create_service<interfaces::srv::DisconnectSource>("disconnect_source", std::bind(&RobotIntrospection::disconnect_source, this, _1, _2));

    timer_= this->create_wall_timer(
		    100ms, std::bind(&RobotIntrospection::timer_loop, this));
  }	

private:
  int robot_id;
  int time_check = 0;
  std::string robot_name;
  float max_battery_voltage = 8.4; // Volts
  float raspberrypi_temperature;
  float microcontroller_temperature;
  std::vector<float> battery_voltage = {0.0, 0.0};
  std::vector<int> battery_charge_percentage = {0, 0};
  std::string battery_supply_status;

  gpiod::chip chip_;
  std::vector<gpiod::line> input_lines_;
  std::vector<gpiod::line> output_lines_;

  std::vector<int> input_pins_ = {
    GPIO_CHECK_CHARGER_CONNECTION,
    GPIO_BATTERY1_LOW_DETECTION,
    GPIO_BATTERY2_LOW_DETECTION,
    GPIO_BATTERY1_FULL_DETECTION,
    GPIO_BATTERY2_FULL_DETECTION
  };

  std::vector<int> output_pins_ = {
    GPIO_DISCONNECT_SOURCE,
    GPIO_RESET_MICROCONTROLLER,
    GPIO_FAN_CONTROLLER
  };
 

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr subscription_;
  rclcpp::Publisher<interfaces::msg::RobotStatus>::SharedPtr publisher_;
  rclcpp::Service<interfaces::srv::ResetMicro>::SharedPtr service_;
  rclcpp::Service<interfaces::srv::TurnOnFan>::SharedPtr fan_service_;
  rclcpp::Service<interfaces::srv::DisconnectSource>::SharedPtr source_service_;

  void auto_toggle() {
    RCLCPP_WARN(this->get_logger(), "RESTARTING MICROCONTROLLER"); 
    output_lines_[1].set_value(false);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    output_lines_[1].set_value(true);
  }

  void read_inputs() {
    std::cout << "GPIO READINGS.-> [";
    for(size_t i = 0; i < input_lines_.size(); i++) {
      std::cout << input_lines_[i].get_value() << ", ";
    }
    std::cout << "]"<< std::endl; 
  }
  
  void state_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
    float min_perc = 0;//%
    float max_perc = 100;//%
    float min_analog = 1900;
    float max_analog = 2650;
    
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
    battery_voltage = {float(max_battery_voltage * battery1_percentage / 100), float(max_battery_voltage * battery2_percentage / 100) };
    battery_charge_percentage = { int(battery1_percentage), int(battery2_percentage)};

    microcontroller_temperature = float(msg->data[21]);
  }

  void reset_micro(const std::shared_ptr<interfaces::srv::ResetMicro::Request>, std::shared_ptr<interfaces::srv::ResetMicro::Response>) {
      auto_toggle();
  }

  void turn_on_fan(const std::shared_ptr<interfaces::srv::TurnOnFan::Request> req, std::shared_ptr<interfaces::srv::TurnOnFan::Response> res) {
    if (req->turn_on) {
      RCLCPP_WARN(this->get_logger(), "TURNING ON FAN");
      output_lines_[2].set_value(true);
    } else {
      RCLCPP_WARN(this->get_logger(), "TURNING OFF FAN");
      output_lines_[2].set_value(false);
    }
  }

  void disconnect_source(const std::shared_ptr<interfaces::srv::DisconnectSource::Request> req, std::shared_ptr<interfaces::srv::DisconnectSource::Response> res) {
    if (req->disconnect) {
      RCLCPP_WARN(this->get_logger(), "DISCONNECTING CHARGER SOURCE");
      output_lines_[0].set_value(true);
    } else {
      RCLCPP_WARN(this->get_logger(), "CONNECTING CHARGER SOURCE");
      output_lines_[0].set_value(true);
    }
  }
  
  void timer_loop() {
    time_check += 1;
    if (time_check >= 50) {
      time_check = 0;
      if (microcontroller_temperature == 0) {
        auto_toggle();
      }
    }
    read_inputs();
    
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
