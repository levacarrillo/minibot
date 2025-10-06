#include "ros_setup.h"
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32_multi_array.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <micro_ros_platformio.h>

#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#error Example for arduino framework with serial transport
#endif

rcl_subscription_t motors_sub;
rcl_publisher_t sensors_pub;

std_msgs__msg__Int32MultiArray sensors_msg;
std_msgs__msg__Float32MultiArray vels_msg;

int32_t sensors_data[8];

rcl_node_t node;
rcl_timer_t timer;
rclc_support_t support;
rclc_executor_t executor;
rcl_allocator_t allocator;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)) {error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop() { while(1) { delay(100); }}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
    RCLC_UNUSED(last_call_time);

    if (timer != NULL) {
        sensors_msg.data.data = sensors_data;
        sensors_msg.data.size = 8;
        sensors_msg.data.capacity = 8;
        RCSOFTCHECK(rcl_publish(&sensors_pub, &sensors_msg, NULL));
    }
}

void subscription_callback(const void * msgin) {
    const std_msgs__msg__Float32MultiArray * msg = (const std_msgs__msg__Float32MultiArray *) msgin;
}

void setup_ros() {
    set_microros_serial_transports(Serial);
    delay(100);
    allocator = rcl_get_default_allocator();
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    RCCHECK(rclc_node_init_default(&node, "microcontroller_node", "", &support));

    RCCHECK(rclc_subscription_init_default(&motors_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), "motors_speed"));
    RCCHECK(rclc_publisher_init_default(&sensors_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray), "sensors"));

    const unsigned int timer_timeout = 1;
    RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(timer_timeout), timer_callback));
    RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
    RCCHECK(rclc_executor_add_timer(&executor, &timer));
    RCCHECK(rclc_executor_add_subscription(&executor, &motors_sub, &vels_msg, &subscription_callback, ON_NEW_DATA));
}

void loop_ros() {
    RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1)));
}
