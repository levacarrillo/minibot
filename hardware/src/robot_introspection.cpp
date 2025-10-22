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
        this->declare_parameter<int>("temperature", this->temperature);
        this->declare_parameter<double>("battery_voltage", this->battery_voltage);
        this->declare_parameter<double>("battery_capacity", this->battery_capacity);
        this->declare_parameter<int>("battery_cells_number", this->battery_cells_number);
        this->declare_parameter<int>("battery_charge_percentage", this->battery_charge_percentage);
        this->declare_parameter<std::string>("battery_supply_status", this->battery_supply_status);        
        
        publisher_ = this->create_publisher<interfaces::msg::RobotStatus>("robot_status", 10);
        
        timer_ = this->create_wall_timer(
            500ms, std::bind(&RobotIntrospection::timer_callback, this)
        );
    }
    private:
    void timer_callback() {
        this->robot_id    = this->get_parameter("robot_id").as_int();
        this->robot_name  = this->get_parameter("robot_name").as_string();
        this->temperature = this->get_parameter("temperature").as_int();
        this->battery_voltage  = this->get_parameter("battery_voltage").as_double();
        this->battery_capacity = this->get_parameter("battery_capacity").as_double();
        this->battery_cells_number = this->get_parameter("battery_cells_number").as_int();
        this->battery_charge_percentage = this->get_parameter("battery_charge_percentage").as_int();
        this->battery_supply_status = this->get_parameter("battery_supply_status").as_string();

        auto message = interfaces::msg::RobotStatus();
        message.robot_id = this->robot_id;
        message.robot_name = this->robot_name;
        //message.temperature = this->temperature;
        // message.battery_voltage = this->battery_voltage;
        // message.battery_capacity = this->battery_capacity;
        // message.battery_cells_number = this->battery_cells_number;
        // message.battery_charge_percentage = this->battery_charge_percentage;
        message.battery_supply_status = this->battery_supply_status;

        publisher_->publish(message);
    }

    int robot_id;
    std::string robot_name;
    int temperature;
    double battery_voltage;
    double battery_capacity;
    int battery_cells_number;
    int battery_charge_percentage;

    std::string battery_supply_status;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<interfaces::msg::RobotStatus>::SharedPtr publisher_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobotIntrospection>());
    rclcpp::shutdown();
    return 0;
}
