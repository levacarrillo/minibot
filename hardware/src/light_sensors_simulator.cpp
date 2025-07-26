#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "interfaces/srv/get_light_readings.hpp"

#include <vector>
#include <cstdlib>
#include <ctime>
#include <memory>
#include <algorithm>
#include <array>


using std::placeholders::_1;

#define ROBOT_RADIUS 0.08

class LightSensorsSimulator : public rclcpp::Node {
public:
    LightSensorsSimulator() : Node("light_sensors_simulator") {
        subscription_ = this->create_subscription<visualization_msgs::msg::Marker>(
            "spot_light_marker", 10, std::bind(&LightSensorsSimulator::topic_callback, this, _1));
        service_ = this->create_service<interfaces::srv::GetLightReadings>(
            "/get_light_readings",
            std::bind(&LightSensorsSimulator::handle_request, this, std::placeholders::_1, std::placeholders::_2)
        );

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),  // 2 Hz
            std::bind(&LightSensorsSimulator::update_readings, this)
        );
        readings_.fill(0.0f);
    }

private:
    void topic_callback(const visualization_msgs::msg::Marker::SharedPtr msg)
    {
        // RCLCPP_INFO(this->get_logger(), "SPOT LIGHT STATE: %s", msg->text.c_str());
        // RCLCPP_INFO(this->get_logger(), "SPOT LIGHT POSITION: x->%f, y->%f", msg->pose.position.x, msg->pose.position.y);
        spot_light_turned_ = (msg->text == "T") ? true : false;
        spot_light_x_ = msg->pose.position.x;
        spot_light_y_ = msg->pose.position.y;
    }

    void update_readings() {
        if (spot_light_turned_) {
            readings_ = { 0.0f, 0.0f, 0.1f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
        } else {
            readings_.fill(0.0f);
        }
    }

    void handle_request(
        const std::shared_ptr<interfaces::srv::GetLightReadings::Request> /*request*/,
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

    bool spot_light_turned_ = false;
    double spot_light_x_, spot_light_y_ = 0.0f;
    std::array<float, 8> readings_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<visualization_msgs::msg::Marker>::SharedPtr subscription_;
    rclcpp::Service<interfaces::srv::GetLightReadings>::SharedPtr service_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LightSensorsSimulator>());
    rclcpp::shutdown();
    return 0;
}
