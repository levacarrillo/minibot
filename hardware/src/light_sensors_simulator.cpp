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


class LightSensorsSimulator : public rclcpp::Node {
public:
    LightSensorsSimulator() : Node("light_sensors_simulator") {
        subscription_ = this->create_subscription<visualization_msgs::msg::Marker>(
            "spot_light_marker", 10, std::bind(&LightSensorsSimulator::topic_callback, this, _1));
        service_ = this->create_service<interfaces::srv::GetLightReadings>(
            "/get_light_readings",
            std::bind(&LightSensorsSimulator::handle_request, this, std::placeholders::_1, std::placeholders::_2)
        );

        // readings_.resize(8, 0.0);
        std::srand(std::time(nullptr));  // Semilla aleatoria

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),  // 2 Hz
            std::bind(&LightSensorsSimulator::update_readings, this)
        );
        readings_.fill(0.0f);
    }

private:
    void topic_callback(const visualization_msgs::msg::Marker::SharedPtr msg) const
    {
        RCLCPP_INFO(this->get_logger(), "SPOT LIGHT STATE: %s", msg->text.c_str());
        RCLCPP_INFO(this->get_logger(), "SPOT LIGHT POSITION: x->%f, y->%f", msg->pose.position.x, msg->pose.position.y);
    }
    void update_readings() {
        // std::cout<<"[";
        // for (auto& val : readings_) {
        //     // val = static_cast<float>(rand() % 100) / 10.0f;
        //     std::cout<<val<<", ";
        // }
        // std::cout<<"]"<<std::endl;
        // RCLCPP_INFO(this->get_logger(), "Lecturas actualizadas.");
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
            response->max_value = *max_it;
            response->max_index = static_cast<int32_t>(std::distance(readings_.begin(), max_it));
        }
    }

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
