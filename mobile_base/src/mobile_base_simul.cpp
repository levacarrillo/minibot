#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"


using namespace std::chrono;

class MobileBaseSimul : public rclcpp::Node {
public:
    MobileBaseSimul() : Node("mobile_base_simul"), x_(0.0), y_(0.0), theta_(0.0) {
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&MobileBaseSimul::cmdVelCallback, this, std::placeholders::_1)
        );

        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&MobileBaseSimul::updatePosition, this)
        );

        last_time_ = this->now();
        last_msg_time_ = steady_clock::now();
    }

private:
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        current_twist_ = *msg;
        last_msg_time_ = steady_clock::now();
    }

    void updatePosition() {
        auto current_time = this->get_clock()->now();
        double dt = (current_time - last_time_).seconds();
        last_time_ = current_time;

        auto now = steady_clock::now();
        auto duration_movement = duration_cast<seconds>(now - last_msg_time_);

        if (duration_movement.count() < 2) {
            double vx = current_twist_.linear.x;
            double wz = current_twist_.angular.z;
    
            x_ += vx * cos(theta_) * dt;
            y_ += vx * sin(theta_) * dt;
            theta_ += wz * dt;
        }
        tf2::Quaternion q;
        q.setRPY(0, 0, theta_);

        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp = this->now();
        odom_msg.header.frame_id = "odom";
        odom_msg.child_frame_id = "base_link";

        odom_msg.pose.pose.position.x = x_;
        odom_msg.pose.pose.position.y = y_;
        odom_msg.pose.pose.position.z = 0.0;
        odom_msg.pose.pose.orientation.x = q.x();
        odom_msg.pose.pose.orientation.y = q.y();
        odom_msg.pose.pose.orientation.z = q.z();
        odom_msg.pose.pose.orientation.w = q.w();

        odom_msg.twist.twist.linear.x = current_twist_.linear.x;
        odom_msg.twist.twist.angular.z = current_twist_.angular.z;
        odom_pub_->publish(odom_msg);
    
        geometry_msgs::msg::TransformStamped transform;

        transform.header.stamp = current_time;
        transform.header.frame_id = "odom";
        transform.child_frame_id = "base_link";

        transform.transform.translation.x = x_;
        transform.transform.translation.y = y_;
        transform.transform.translation.z = 0.0;

        transform.transform.rotation.x = q.x();
        transform.transform.rotation.y = q.y();
        transform.transform.rotation.z = q.z();
        transform.transform.rotation.w = q.w();

        tf_broadcaster_->sendTransform(transform);


    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    steady_clock::time_point last_msg_time_;

    geometry_msgs::msg::Twist current_twist_;
    rclcpp::Time last_time_;
    double x_, y_, theta_;
};
    
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MobileBaseSimul>());
    rclcpp::shutdown();
    return 0;
}
