#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "interfaces/srv/odom_set_point.hpp"


#define BASE_WIDTH 0.095 // METERS
#define TICKS_PER_METER 4442.0
#define WHEEL_DIAMETER  0.04338 // METERS 


using namespace std::chrono;
using OdomSetPoint = interfaces::srv::OdomSetPoint;

class MobileBase : public rclcpp::Node {

public:
  MobileBase() : Node("mobile_base") {
    std::cout << "INITIALIZING mobile_base NODE BY Luis Gonzalez" << std::endl;     
    
    odom_pub_= this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);

    tf_broadcaster_= std::make_shared<tf2_ros::TransformBroadcaster>(this);

    motors_pub_= this->create_publisher<std_msgs::msg::Float32MultiArray>("motors_speed", 10);
        
    odom_service_= this->create_service<OdomSetPoint>(
		    "odom_set_point",
		    std::bind(&MobileBase::SetPoint, this, std::placeholders::_1, std::placeholders::_2));

    encoders_sub_= this->create_subscription<std_msgs::msg::Int32MultiArray>(
		    "/sensors",
		    10,
		    std::bind(&MobileBase::encodersCallback, this, std::placeholders::_1));
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
		    "/cmd_vel",
		    10,
		    std::bind(&MobileBase::cmdVelCallback, this, std::placeholders::_1));

    vels_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("velocity_for_check", 10);
  }	

private:
  bool once_flag = true;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  rclcpp::Service<interfaces::srv::OdomSetPoint>::SharedPtr odom_service_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr encoders_sub_;
  
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr motors_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr vels_pub_;


  float left_speed  = 0.0;
  float right_speed = 0.0;
  long last_encoder_left;
  long last_encoder_right;
  double left_distance = 0, right_distance = 0;
  double robot_x = 0.0, robot_y = 0.0, robot_t = 0.0;
  double curr_linear_vel_x = 0.0, curr_linear_vel_y = 0.0, curr_angular_vel = 0.0;
  geometry_msgs::msg::Twist goal_vel;

  void SetPoint(const std::shared_ptr<OdomSetPoint::Request>,
		      std::shared_ptr<OdomSetPoint::Response> response)
  {
    once_flag = true;
    robot_t = 0.0;
    robot_x = 0.0;
    robot_y = 0.0;
    response->done = true;
  }

  void encodersCallback(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
    double left_count  = msg->data[22];
    double right_count = msg->data[23];
    double curr_vel_left  = (M_PI * WHEEL_DIAMETER * msg->data[26]) / 60;
    double curr_vel_right = (M_PI * WHEEL_DIAMETER * msg->data[27]) / 60;

    if (left_speed < 0) {
        curr_vel_left = - curr_vel_left;
    }
    if (right_speed < 0) {
    	curr_vel_right = -curr_vel_right;
    }

    if (once_flag) {
      last_encoder_left  = left_count;
      last_encoder_right = right_count;
      left_distance  = 0;
      right_distance = 0; 
      once_flag = false; 
    }
 
    // RCLCPP_INFO(this->get_logger(), "encodersCallback.msg->[%d, %d]", msg->data[0], msg->data[1]);
    double dist_left  = (left_count  - last_encoder_left)  / TICKS_PER_METER;
    double dist_right = (right_count - last_encoder_right) / TICKS_PER_METER;
    
    double delta_theta = (dist_right - dist_left) / BASE_WIDTH;
    left_distance  += dist_left;
    right_distance += dist_right; 
    double dist_x  = (dist_right + dist_left) / 2.0;

    robot_t = normalizeAngle(delta_theta + robot_t);
    robot_x += dist_x * cos(robot_t);
    robot_y += dist_x * sin(robot_t);
    //RCLCPP_INFO(this->get_logger(), "ROBOT POSITION: X->%f, Y->%f, ANGLE->%f", robot_x, robot_y, robot_t);

    last_encoder_left  = left_count;
    last_encoder_right = right_count;

    curr_linear_vel_x = cos(robot_t) * (curr_vel_left + curr_vel_right) / 2.0;
    curr_linear_vel_y = sin(robot_t) * (curr_vel_left + curr_vel_right) / 2.0;
    curr_angular_vel = (curr_vel_right - curr_vel_left) / BASE_WIDTH;
    // RCLCPP_INFO(this->get_logger(), "CURRENT VEL: Vx->%f, Vy->%f, ANGULAR->%f", curr_linear_vel_x, curr_linear_vel_y, curr_angular_vel);

    auto message = std_msgs::msg::Float32MultiArray();
    message.data.resize(6);
    message.data[0] = goal_vel.linear.x;
    message.data[1] = goal_vel.linear.y;
    message.data[2] = goal_vel.angular.z;
    message.data[3] = curr_linear_vel_x;
    message.data[4] = curr_linear_vel_y;
    message.data[5] = curr_angular_vel;

    vels_pub_->publish(message);
    updatePosition();
  }

  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    // RCLCPP_INFO(this->get_logger(), "VELS: LINEAR-> %f, ANGULAR-> %f", msg->linear.x, msg->angular.z);
    goal_vel.linear.x = msg->linear.x;
    goal_vel.linear.y = msg->linear.y;
    goal_vel.angular.z = msg->angular.z;
    right_speed = msg->linear.x + msg->angular.z * BASE_WIDTH / 2.0;
    left_speed  = msg->linear.x - msg->angular.z * BASE_WIDTH / 2.0;

    auto message = std_msgs::msg::Float32MultiArray();
    message.data.resize(2);
    message.data[0] = left_speed;
    message.data[1] = right_speed;
    // rclcpp_info(this->get_logger(), "publishing floats->[%f, %f]", left_speed, right_speed);
    motors_pub_->publish(message);
  }

  double normalizeAngle(double angle) {
    /*if (angle > M_PI * 2) {
      return fmod(angle, (M_PI * 2));
    } else if (angle < 0) {
      return angle * M_PI * 2 - fmod(-angle, (M_PI * 2));
    } */
    return angle;
  }

  void updatePosition() {
    // RCLCPP_INFO(this->get_logger(), "WHEELS DIS: LEFT->%f, RIGHT->%f", left_distance, right_distance);
    tf2::Quaternion q;
    q.setRPY(0, 0, robot_t);

    nav_msgs::msg::Odometry odom_msg; 
    odom_msg.header.stamp = this->now();
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_link";

    odom_msg.pose.pose.position.x = robot_x;
    odom_msg.pose.pose.position.y = robot_y;
    odom_msg.pose.pose.position.z = 0.0;
    odom_msg.pose.pose.orientation.x = q.x();
    odom_msg.pose.pose.orientation.y = q.y();
    odom_msg.pose.pose.orientation.z = q.z();
    odom_msg.pose.pose.orientation.w = q.w();

    odom_msg.twist.twist.linear.x = curr_linear_vel_x;
    odom_msg.twist.twist.linear.y = curr_linear_vel_y;
    odom_msg.twist.twist.angular.z = curr_angular_vel;

    odom_pub_->publish(odom_msg);

   
    geometry_msgs::msg::TransformStamped transform;

    transform.header.stamp = this->now();
    transform.header.frame_id = "odom";
    transform.child_frame_id  = "base_link";

    transform.transform.translation.x = robot_x;
    transform.transform.translation.y = robot_y;
    transform.transform.translation.z = 0.0;
 
    transform.transform.rotation.x = q.x();
    transform.transform.rotation.y = q.y();
    transform.transform.rotation.z = q.z();
    transform.transform.rotation.w = q.w();

    tf_broadcaster_->sendTransform(transform);
  }
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MobileBase>());
  rclcpp::shutdown();

  return 0;
}
