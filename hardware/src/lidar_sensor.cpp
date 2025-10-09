#include "rclcpp/rclcpp.hpp"
#include "interfaces/srv/get_scan.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"


using namespace std::placeholders;


class LidarSensor: public rclcpp::Node {
    public:
        LidarSensor() : Node("lidar_sensor") {
	    RCLCPP_INFO(this->get_logger(), "INITIALIZING lidar_sensor node by Luis Gonzalez...");
	    readings_sub_= this->create_subscription<std_msgs::msg::Int32MultiArray>(
			    "sensors", 10, std::bind(&LidarSensor::sharpsCallback, this, _1));
            publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan", 10);
            service_ = this->create_service<interfaces::srv::GetScan>(
                "/get_scan",
                std::bind(&LidarSensor::handle_get_scan, this, _1, _2)
            );

            timer_ = this->create_wall_timer(
                std::chrono::milliseconds(100),
                std::bind(&LidarSensor::simulate_scan, this)
            );
        }

    private:
        float min_value = 0.04f;
        float max_value = 0.3f;
        float angle_min = - 1.5708f;
        float angle_max =   1.5708f;
        float angle_increment = 0.01745f * 5;
        float robot_radius = 0.0635f;
        bool map_loaded_ = false;
	rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr readings_sub_;
        rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
        rclcpp::Service<interfaces::srv::GetScan>::SharedPtr service_;
        rclcpp::TimerBase::SharedPtr timer_;
        sensor_msgs::msg::LaserScan last_scan_;
        std::vector<float> laser_scan;

	void sharpsCallback(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
            // RCLCPP_INFO(this->get_logger(), "DATA RECEIVED-> %s", msg->data;
	}

        void simulate_scan() {
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
	    for(size_t i=0; i < 3; ++i) {
	        laser_scan.push_back(0.1);
	    } 

            publisher_->publish(msg);
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
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarSensor>());
    rclcpp::shutdown();

    return 0;
}
