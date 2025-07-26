#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "visualization_msgs/msg/marker.hpp"


class SpotlightMarker : public rclcpp::Node
{
public:
    SpotlightMarker() : Node("spotlight_marker")
    {
        publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("spotlight_state", 10);

        this->declare_parameter("spotlight_state", true);
        this->declare_parameter("position_x", 2.20);
        this->declare_parameter("position_y", 0.75);

        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&SpotlightMarker::timer_callback, this));
    }

private:
    void timer_callback()
    {
        bool spot_light_turned = this->get_parameter("spotlight_state").as_bool();
        position_x_ = this->get_parameter("position_x").as_double();
        position_y_ = this->get_parameter("position_y").as_double();

        auto marker = visualization_msgs::msg::Marker();
        marker.header.frame_id = "map";
        marker.header.stamp = this->get_clock()->now();
        marker.ns = "spotlight";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = position_x_;
        marker.pose.position.y = position_y_;
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
            marker.text = 'T';
        }
        else
        {
            marker.color.r = 1.0;
            marker.color.g = 1.0;
            marker.color.b = 1.0;
            marker.color.a = 0.7;
            marker.text = 'F';
        }

        publisher_->publish(marker);

        geometry_msgs::msg::TransformStamped transform;

        transform.header.stamp = this->get_clock()->now();
        transform.header.frame_id = "map";
        transform.child_frame_id = "spotlight";

        transform.transform.translation.x = position_x_;
        transform.transform.translation.y = position_y_;
        transform.transform.translation.z = 0.3;

        tf_broadcaster_->sendTransform(transform);

    }

    double position_x_, position_y_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SpotlightMarker>());
    rclcpp::shutdown();
    return 0;
}