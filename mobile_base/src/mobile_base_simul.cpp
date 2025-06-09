#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"

class MobileBaseSimul : public rclcpp::Node {
public:
    MobileBaseSimul() : Node("mobile_base_simul"), x_(0.0), y_(0.0), theta_(0.0) {
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
    }

private:
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        current_twist_ = *msg;
    }

    void updatePosition() {
        auto current_time = this->now();
        double dt = (current_time - last_time_).seconds();
        last_time_ = current_time;

        double vx = current_twist_.linear.x;
        double wz = current_twist_.angular.z;

        x_ += vx * cos(theta_) * dt;
        y_ += vx * sin(theta_) * dt;
        theta_ += wz * dt;

        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = current_time;
        transform.header.frame_id = "odom";
        transform.child_frame_id = "base_link";

        transform.transform.translation.x = x_;
        transform.transform.translation.y = y_;
        transform.transform.translation.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0, 0, theta_);
        transform.transform.rotation.x = q.x();
        transform.transform.rotation.y = q.y();
        transform.transform.rotation.z = q.z();
        transform.transform.rotation.w = q.w();

        tf_broadcaster_->sendTransform(transform);
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;

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
