#include <memory>
#include <vector>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "interfaces/srv/get_scan.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <random_numbers/random_numbers.h>


using namespace random_numbers;
using namespace std::placeholders;


class LidarSimulator : public rclcpp::Node {
    public:
        LidarSimulator() : Node("lidar_simulator") {
            publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan", 10);
            map_sub_   = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
                "/map", 10, std::bind(&LidarSimulator::map_callback, this, _1));
            service_ = this->create_service<interfaces::srv::GetScan>(
                "/get_scan",
                std::bind(&LidarSimulator::handle_get_scan, this, _1, _2)
            );
            tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
            tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
            tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

            this->make_transforms();

            timer_ = this->create_wall_timer(
                std::chrono::milliseconds(100),
                std::bind(&LidarSimulator::simulate_scan, this)
            );
        }

    private:
        float min_value = 0.04f;
        float max_value = 0.3f;
        float angle_min = - 1.5708f;
        float angle_max =   1.5708f;
        float angle_increment = 0.01745f * 5;
        float robot_radius = 0.0635f;
        RandomNumberGenerator rng;
        nav_msgs::msg::OccupancyGrid map_;
        bool map_loaded_ = false;
        rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
        rclcpp::Service<interfaces::srv::GetScan>::SharedPtr service_;
        rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
        rclcpp::TimerBase::SharedPtr timer_;
        sensor_msgs::msg::LaserScan last_scan_;
        std::vector<float> laser_scan;
        std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

        void simulate_scan() {
            if (!map_loaded_) return;

            geometry_msgs::msg::TransformStamped t;
            std::string fromFrameRel = "map";
            std::string toFrameRel = "base_link";
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
            RCLCPP_INFO(this->get_logger(), "ROBOT POSITION: x->%f, y->%f, angle->%f", t.transform.translation.x, t.transform.translation.y, yaw);

            auto msg = sensor_msgs::msg::LaserScan();

            msg.header.stamp = this->get_clock()->now();
            msg.header.frame_id = "laser_frame";
            msg.angle_min = this->angle_min;
            msg.angle_max = this->angle_max;
            msg.angle_increment = this->angle_increment;
            msg.range_min = this->robot_radius + this->min_value;
            msg.range_max = this->robot_radius + this->max_value;

            size_t num_readings = static_cast<size_t>((msg.angle_max - msg.angle_min) / msg.angle_increment);
            msg.ranges.resize(num_readings);

            laser_scan.clear();
            for (size_t i = 0; i < num_readings; ++i) {
                msg.ranges[i] = get_simulated_ray(this->max_value + this->rng.gaussian(0.0, 0.01));
                laser_scan.push_back(msg.ranges[i]);
            }

            last_scan_ = msg;
            publisher_->publish(msg);
        }

        void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
            map_ = *msg;
            map_loaded_ = true;
        }

        void handle_get_scan(
            const std::shared_ptr<interfaces::srv::GetScan::Request>,
            std::shared_ptr<interfaces::srv::GetScan::Response> response
        ) {
            response->scan = laser_scan;
            response->angle_min = this->angle_min;
            response->angle_max = this->angle_max;
            response->max_value = this->robot_radius + this->max_value;
        }

        void make_transforms() {
            geometry_msgs::msg::TransformStamped t;

            t.header.stamp = this->get_clock()->now();
            t.header.frame_id = "base_link";
            t.child_frame_id = "laser_frame";

            t.transform.translation.x = 0.00;
            t.transform.translation.y = 0.00;
            t.transform.translation.z = 0.05;
            tf2::Quaternion q;
            q.setRPY(0, 0, 0);
            t.transform.rotation.x = q.x();
            t.transform.rotation.y = q.y();
            t.transform.rotation.z = q.z();
            t.transform.rotation.w = q.w();

            tf_static_broadcaster_->sendTransform(t);
        }

        double get_simulated_ray(double ray) {
            return ray;
        }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarSimulator>());
    rclcpp::shutdown();
    return 0;
}
