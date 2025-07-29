#include "rclcpp/rclcpp.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker.hpp"
#include "interfaces/srv/get_light_readings.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

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
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        subscription_ = this->create_subscription<visualization_msgs::msg::Marker>(
            "spotlight_state", 10, std::bind(&LightSensorsSimulator::topic_callback, this, _1));
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
        spotlight_turned_ = (msg->text == "T") ? true : false;
    }

    void update_readings() {
        if (spotlight_turned_) {
            geometry_msgs::msg::TransformStamped t;
            std::string fromFrameRel = "base_link";
            std::string toFrameRel = "spotlight";
            try {
                t = tf_buffer_->lookupTransform(toFrameRel, fromFrameRel, tf2::TimePointZero);
            } catch (const tf2::TransformException & ex) {
                RCLCPP_INFO(
                    this->get_logger(), "Could not transform %s to %s: %s",
                    toFrameRel.c_str(), fromFrameRel.c_str(), ex.what());
            }
            geometry_msgs::msg::Quaternion quaternion = t.transform.rotation;
            tf2::Quaternion tf2_quat(quaternion.x, quaternion.y, quaternion.z, quaternion.w);
            double roll, pitch, yaw;
            tf2::Matrix3x3(tf2_quat).getRPY(roll, pitch, yaw);
            // RCLCPP_INFO(this->get_logger(), "DISTANCE TO SPOTLIGHT: x->%f, y->%f, angle->%f", t.transform.translation.x, t.transform.translation.y, yaw);

            for (int i=0; i<8; i++) {
                float sensor_distance_x = t.transform.translation.x + ROBOT_RADIUS * std::cos(yaw + i * M_PI / 4);
                float sensor_distance_y = t.transform.translation.y + ROBOT_RADIUS * std::sin(yaw + i * M_PI / 4);
                readings_[i] = 1 / std::hypot(sensor_distance_x, sensor_distance_y);
            }
            // readings_ = { 0.0f, 0.0f, 0.1f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
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

    bool spotlight_turned_ = false;
    std::array<float, 8> readings_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<visualization_msgs::msg::Marker>::SharedPtr subscription_;
    rclcpp::Service<interfaces::srv::GetLightReadings>::SharedPtr service_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LightSensorsSimulator>());
    rclcpp::shutdown();
    return 0;
}
