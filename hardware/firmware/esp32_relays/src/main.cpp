#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32_multi_array.h>


#define BAUDRATE 115200

#define RELAY_1 2
#define RELAY_2 4

#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#error This example is only avaliable for Arduino framework with serial transport.
#endif

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;


rcl_subscription_t relays_sub;

std_msgs__msg__Int32MultiArray relays_msg;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop() {
  while(1) {
    delay(100);
  }
}

void subscription_callback(const void * msgin) {
  const std_msgs__msg__Int32MultiArray * msg = (const std_msgs__msg__Int32MultiArray *) msgin;

  if(msg->data.data[0] == 1) digitalWrite(RELAY_1, HIGH);
  else digitalWrite(RELAY_1, LOW);

  if(msg->data.data[1] == 1) digitalWrite(RELAY_2, HIGH);
  else digitalWrite(RELAY_2, LOW);

}

void setup() {
  Serial.begin(BAUDRATE);
  set_microros_serial_transports(Serial);
  delay(2000);

  int relays_buffer[2];
  relays_msg.data.data = relays_buffer;
  relays_msg.data.size = 0;
  relays_msg.data.capacity = 2;

  allocator = rcl_get_default_allocator();

  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "relays_controller_node", "", &support));

  RCCHECK(rclc_subscription_init_default(&relays_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray), "relays_state"));
  
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &relays_sub, &relays_msg, &subscription_callback, ON_NEW_DATA));

  pinMode(RELAY_1, OUTPUT);
  pinMode(RELAY_2, OUTPUT);
}

void loop() {
  delay(100);
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}