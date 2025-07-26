#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"

class SpotLightsMarker : public rclcpp::Node
{
public:
    SpotLightsMarker() : Node("spot_lights_marker")
    {
        publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("spot_light_marker", 10);

        this->declare_parameter("spot_light_turned", true);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&SpotLightsMarker::timer_callback, this));
    }

private:
    void timer_callback()
    {
        bool spot_light_turned = this->get_parameter("spot_light_turned").as_bool();

        auto marker = visualization_msgs::msg::Marker();
        marker.header.frame_id = "map";
        marker.header.stamp = this->get_clock()->now();
        marker.ns = "spot_light";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = 2.20;
        marker.pose.position.y = 0.75;
        marker.pose.position.z = 0.3;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.12;
        marker.scale.y = 0.12;
        marker.scale.z = 0.12;

        if (spot_light_turned)
        {
            marker.color.r = 1.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
            marker.color.a = 1.0;
        }
        else
        {
            marker.color.r = 1.0;
            marker.color.g = 1.0;
            marker.color.b = 1.0;
            marker.color.a = 0.7;
        }

        publisher_->publish(marker);
    }

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SpotLightsMarker>());
    rclcpp::shutdown();
    return 0;
}