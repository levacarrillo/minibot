#include "rclcpp/rclcpp.hpp"
#include "interfaces/msg/robot_status.hpp"
#include <chrono>
#include <memory>

using namespace std::chrono_literals;

class RobotIntrospection : public rclcpp::Node {
public:
    RobotIntrospection() : Node ("robot_introspection") {
        RCLCPP_INFO(this->get_logger(), "INITIALIZING ROBOT_INTROSPECTION NODE...");
        this->declare_parameter<int>("robot_id", this->robot_id);
        this->declare_parameter<std::string>("robot_name", this->robot_name);
        this->declare_parameter<float>("microcontroller_temperature", this->microcontroller_temperature);
        this->declare_parameter<float>("raspberrypi_temperature", this->raspberrypi_temperature);
        this->declare_parameter<std::vector<float>>("battery_voltage", this->battery_voltage);
        this->declare_parameter<std::vector<int32_t>>("battery_charge_percentage", this->battery_charge_percentage);
        this->declare_parameter<std::string>("battery_supply_status", this->battery_supply_status);        
        
        publisher_ = this->create_publisher<interfaces::msg::RobotStatus>("robot_status", 10);
        
        timer_ = this->create_wall_timer(
            100ms, std::bind(&RobotIntrospection::timer_callback, this)
        );
    }

    private:
    int robot_id = 0;
    std::string robot_name = "NO SELECTED";
    float microcontroller_temperature = 0.0;
    float raspberrypi_temperature = 0.0;
    std::vector<float> battery_voltage = {0.0, 0.0};
    std::vector<int32_t> battery_charge_percentage = {0, 0};
    std::string battery_supply_status = "NO DEFINED";
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<interfaces::msg::RobotStatus>::SharedPtr publisher_;

    void timer_callback() {
        this->robot_id    = this->get_parameter("robot_id").as_int();
        this->robot_name  = this->get_parameter("robot_name").as_string();
        this->microcontroller_temperature = this->get_parameter("microcontroller_temperature").as_double();
        this->raspberrypi_temperature = this->get_parameter("raspberrypi_temperature").as_double();
        auto battery_voltage = this->get_parameter("battery_voltage").as_double_array();
        auto battery_charge_percentage  = this->get_parameter("battery_charge_percentage").as_integer_array();
        this->battery_supply_status = this->get_parameter("battery_supply_status").as_string();

        this->battery_voltage.clear();
        this->battery_charge_percentage.clear();
        this->battery_voltage.reserve(battery_voltage.size());
        this->battery_charge_percentage.reserve(battery_charge_percentage.size());

        for (auto &v : battery_voltage) {
            this->battery_voltage.push_back(static_cast<double>(v));
        }
        for (auto &v : battery_charge_percentage) {
            this->battery_charge_percentage.push_back(static_cast<int32_t>(v));
        }

        auto message = interfaces::msg::RobotStatus();
        message.robot_id = this->robot_id;
        message.robot_name = this->robot_name;
        message.microcontroller_temperature = this->microcontroller_temperature;
        message.raspberrypi_temperature = this->raspberrypi_temperature;
        message.battery_voltage = this->battery_voltage;
        message.battery_charge_percentage = this->battery_charge_percentage;
        message.battery_supply_status = this->battery_supply_status;

        publisher_->publish(message);
    }
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobotIntrospection>());
    rclcpp::shutdown();
    return 0;
}
