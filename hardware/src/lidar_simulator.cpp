#include <memory>
#include <vector>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "interfaces/srv/get_scan.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"


class LidarSimulator : public rclcpp::Node
{
public:
    LidarSimulator() : Node("lidar_simulator")
    {
        publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan", 10);
        service_ = this->create_service<interfaces::srv::GetScan>(
            "/get_scan",
            std::bind(&LidarSimulator::handle_get_scan, this, std::placeholders::_1, std::placeholders::_2)
        );
        tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

        this->make_transforms();

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&LidarSimulator::publish_scan, this)
        );
    }

private:
    void publish_scan()
    {
        auto msg = sensor_msgs::msg::LaserScan();

        msg.header.stamp = this->get_clock()->now();
        msg.header.frame_id = "laser_frame";
        msg.angle_min = -1.57;
        msg.angle_max = 1.57;
        msg.angle_increment = 0.01;
        msg.range_min = 0.2;
        msg.range_max = 6.0;

        size_t num_readings = static_cast<size_t>((msg.angle_max - msg.angle_min) / msg.angle_increment);
        msg.ranges.resize(num_readings);

        for (size_t i = 0; i < num_readings; ++i) {
            msg.ranges[i] = 3.0 + std::sin(i * 0.1);
        }

        last_scan_ = msg;
        publisher_->publish(msg);
    }

    void handle_get_scan(
        const std::shared_ptr<interfaces::srv::GetScan::Request>,
        std::shared_ptr<interfaces::srv::GetScan::Response> response
    ) {
        response->scan = last_scan_;
    }

    void make_transforms() {
        geometry_msgs::msg::TransformStamped t;

        t.header.stamp = this->get_clock()->now();
        t.header.frame_id = "base_link";
        t.child_frame_id = "laser_frame";

        t.transform.translation.x = 0;
        t.transform.translation.y = 0;
        t.transform.translation.z = 0;
        tf2::Quaternion q;
        q.setRPY(
        0,
        0,
        0);
        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();

        tf_static_broadcaster_->sendTransform(t);
    }

    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
    rclcpp::Service<interfaces::srv::GetScan>::SharedPtr service_;
    rclcpp::TimerBase::SharedPtr timer_;
    sensor_msgs::msg::LaserScan last_scan_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarSimulator>());
    rclcpp::shutdown();
    return 0;
}
