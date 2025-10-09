#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "interfaces/srv/get_light_readings.hpp"


class LightSensors : public rclcpp::Node {
public:
    LightSensors() : Node("light_sensors") {
	RCLCPP_INFO(this->get_logger(), "INITIALIZING light_sensors node by Luis Gonzalez...");
        light_sensors_sub_ = this->create_subscription<std_msgs::msg::Int32MultiArray>("sensors", 10, std::bind(&LightSensors::light_callback, this, std::placeholders::_1));
        service_ = this->create_service<interfaces::srv::GetLightReadings>(
            "/get_light_readings",
            std::bind(&LightSensors::handle_request, this, std::placeholders::_1, std::placeholders::_2)
        );
        readings_.fill(0.0f);
    }

private:
    void light_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
	for (size_t i = 0; i < readings_.size() && i < msg->data.size(); ++i) {
	    readings_[i] = static_cast<float>(msg->data[i]);
	}
    }

    void handle_request(
        const std::shared_ptr<interfaces::srv::GetLightReadings::Request>,
        std::shared_ptr<interfaces::srv::GetLightReadings::Response> response
    ) {
        auto max_it = std::max_element(readings_.begin(), readings_.end());
        if (readings_.empty() || *max_it <= 0.0f) {
            response->max_index = -1;
            response->max_value = std::numeric_limits<float>::quiet_NaN();
        }
        else {
            response->readings  = readings_;
            response->max_value = *max_it;
            response->max_index = static_cast<int32_t>(std::distance(readings_.begin(), max_it));
        }
    }

    std::array<float, 8> readings_;
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr light_sensors_sub_;
    rclcpp::Service<interfaces::srv::GetLightReadings>::SharedPtr service_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LightSensors>());
    rclcpp::shutdown();
    return 0;
}
