#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/static_transform_broadcaster.h"

class StaticTFMapToOdom : public rclcpp::Node {
public:
    StaticTFMapToOdom() : Node("static_tf_map_to_odom") {
        broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

        geometry_msgs::msg::TransformStamped tf;
        tf.header.stamp = this->get_clock()->now();
        tf.header.frame_id = "map";
        tf.child_frame_id = "odom";

        tf.transform.translation.x = 0.5;
        tf.transform.translation.y = 0.5;
        tf.transform.translation.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0, 0, 0);
        tf.transform.rotation.x = q.x();
        tf.transform.rotation.y = q.y();
        tf.transform.rotation.z = q.z();
        tf.transform.rotation.w = q.w();

        broadcaster_->sendTransform(tf);
        RCLCPP_INFO(this->get_logger(), "Static TF map → odom published");
    }

private:
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> broadcaster_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StaticTFMapToOdom>());
    rclcpp::shutdown();
    return 0;
}
