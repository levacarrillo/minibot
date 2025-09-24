
#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32_multi_array.h>
#include <std_msgs/msg/float32_multi_array.h>


#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#error This example is only avaliable for Arduino framework with serial transport.
#endif

#define BAUDRATE       3000000

#define STOP_BUTTON         39

#define RIGHT_LINE_SENSOR    2
#define LEFT_LINE_SENSOR    37
#define CENTER_LINE_SENSOR  42

#define RH_ENCODER_A 20
#define RH_ENCODER_B 19
#define LH_ENCODER_A 35
#define LH_ENCODER_B 47

#define PWM_FREQ         5000
#define PWM_RESOLUTION      8
#define PWM_CHANNEL_LEFT1   0
#define PWM_CHANNEL_LEFT2   1
#define PWM_CHANNEL_RIGHT1  2
#define PWM_CHANNEL_RIGHT2  3

#define MOTOR_LEFT_INI1   40
#define MOTOR_LEFT_INI2   41
#define MOTOR_RIGHT_INI3  38
#define MOTOR_RIGHT_INI4  21


rcl_subscription_t motors_sub;

rcl_publisher_t line_publisher,
                light_publisher,
                sharp_publisher,
                state_publisher,
                enconders_publisher;

std_msgs__msg__Int32MultiArray line_msg;
std_msgs__msg__Int32MultiArray state_msg;
std_msgs__msg__Int32MultiArray light_msg;
std_msgs__msg__Int32MultiArray sharp_msg;
std_msgs__msg__Int32MultiArray encoders_msg;
std_msgs__msg__Float32MultiArray vels_msg;


unsigned int pulses_per_turn = 600;
volatile long delta_count[2] = {0, 0};
volatile long encoder_count[2];

float P[2] = { 4.0, 4.0 };
float I[2] = { 0.005, 0.005 };
float D[2] = { 0.0, 0.0 };


double sampling_time = 40; // milliseconds
double sampling_time_sec = sampling_time / 1000.0;
unsigned int previous_time = millis();


int32_t line_data[8];
int32_t light_data[8];
int32_t sharp_data[8];
int32_t state_data[6];
int32_t encoders_data[2];

int max_rpm = 350;
int LEFT = 0, RIGHT = 1;

float wheel_diameter = (43.38) / 1000;
float goal_rpm[2];
float curr_rpm[2];
float goal_speed[2];
float pid_output[2];
float last_pid_output[2] = {0.0, 0.0};

float rpm_error_1[2] = { 0.0, 0.0 };
float rpm_error_2[2] = { 0.0, 0.0 };


rcl_node_t  node;
rcl_timer_t timer;
rclc_support_t  support;
rclc_executor_t executor;
rcl_allocator_t allocator;


#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}


void error_loop() { while(1) { delay(100); }}

void IRAM_ATTR left_encoder_event() {
  if (digitalRead(LH_ENCODER_A) == HIGH) {
    if (digitalRead(LH_ENCODER_B) == LOW) encoder_count[LEFT]--;
    encoder_count[LEFT]++;
  } else {
    if (digitalRead(LH_ENCODER_B) == LOW) encoder_count[LEFT]++;
    else encoder_count[LEFT]--;
  }
}

void IRAM_ATTR right_encoder_event() {
  if (digitalRead(RH_ENCODER_A) == HIGH) {
    if (digitalRead(RH_ENCODER_B) == LOW) encoder_count[RIGHT]++;
    else encoder_count[RIGHT]--;
  } else {
    if (digitalRead(RH_ENCODER_B) == LOW) encoder_count[RIGHT]--;
    else encoder_count[RIGHT]++;
  }
}

void subscription_callback(const void * msgin) {
    const std_msgs__msg__Float32MultiArray * msg = (const std_msgs__msg__Float32MultiArray *)msgin;
    goal_speed[LEFT]  = msg->data.data[0];
    goal_speed[RIGHT] = msg->data.data[1];
    goal_rpm[LEFT]  = fabs((msg->data.data[0] * 60) / (M_PI * wheel_diameter));
    goal_rpm[RIGHT] = fabs((msg->data.data[1] * 60) / (M_PI * wheel_diameter));
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);

  if (timer != NULL) {
    unsigned long current_time = millis();
    double elapsed_time = (double) (current_time - previous_time);
    if (elapsed_time >= sampling_time) {
      delta_count[LEFT]  = encoder_count[LEFT]  - delta_count[LEFT];
      delta_count[RIGHT] = encoder_count[RIGHT] - delta_count[RIGHT];

      curr_rpm[LEFT]  = 60000 * fabs(delta_count[LEFT])  / (pulses_per_turn * elapsed_time);
      curr_rpm[RIGHT] = 60000 * fabs(delta_count[RIGHT]) / (pulses_per_turn * elapsed_time);

      delta_count[LEFT]  = encoder_count[LEFT];
      delta_count[RIGHT] = encoder_count[RIGHT];
      previous_time = current_time;

      float error_rpm[2] = {goal_rpm[LEFT]  - curr_rpm[LEFT],
                            goal_rpm[RIGHT] - curr_rpm[RIGHT]};


      // COMPUTING PID VALUES
      pid_output[LEFT]  = last_pid_output[LEFT] + 
                          (P[LEFT] + D[LEFT] / elapsed_time) * error_rpm[LEFT] +
                          (-P[LEFT] + I[LEFT] * elapsed_time - 2 * D[LEFT] / elapsed_time) * rpm_error_1[LEFT] +
                          (D[LEFT] / sampling_time) * rpm_error_2[LEFT];

      pid_output[RIGHT] = last_pid_output[RIGHT] + 
                          (P[RIGHT] + D[RIGHT] / sampling_time) * error_rpm[RIGHT] +
                          (-P[RIGHT] + I[RIGHT] * elapsed_time - 2 * D[RIGHT] / elapsed_time) * rpm_error_1[RIGHT] +
                          (D[RIGHT] / sampling_time) * rpm_error_2[RIGHT];

      last_pid_output[LEFT]  = pid_output[LEFT];
      last_pid_output[RIGHT] = pid_output[RIGHT];

      rpm_error_2[LEFT]  = rpm_error_1[LEFT];
      rpm_error_2[RIGHT] = rpm_error_1[RIGHT];
      rpm_error_1[LEFT]  = error_rpm[LEFT];
      rpm_error_1[RIGHT] = error_rpm[RIGHT];

      int pwm[2] = {
                    map(pid_output[LEFT],  0, max_rpm, 0, 255), 
                    map(pid_output[RIGHT], 0, max_rpm, 0, 255)
                  };
    
      if (goal_speed[LEFT] == 0) {
        ledcWrite(PWM_CHANNEL_LEFT1, 0);
        ledcWrite(PWM_CHANNEL_LEFT2, 0);
      } else {
        if (goal_speed[LEFT] < 0) {
          ledcWrite(PWM_CHANNEL_LEFT1, pwm[LEFT]);
          ledcWrite(PWM_CHANNEL_LEFT2, 0);
        } else {
          ledcWrite(PWM_CHANNEL_LEFT1, 0);
          ledcWrite(PWM_CHANNEL_LEFT2, pwm[LEFT]);
        }
      }

      if (goal_speed[RIGHT] == 0) {
        ledcWrite(PWM_CHANNEL_RIGHT1, 0);
        ledcWrite(PWM_CHANNEL_RIGHT2, 0);
      } else {
        if (goal_speed[RIGHT] < 0) {
          ledcWrite(PWM_CHANNEL_RIGHT1, pwm[RIGHT]);
          ledcWrite(PWM_CHANNEL_RIGHT2, 0);
        } else {
          ledcWrite(PWM_CHANNEL_RIGHT1, 0);
          ledcWrite(PWM_CHANNEL_RIGHT2, pwm[RIGHT]);
        }
      }
    }

    light_data[0] = analogRead(18);
    light_data[1] = analogRead(17);
    light_data[2] = analogRead(16);
    light_data[3] = analogRead(15);
    light_data[4] = analogRead(4);
    light_data[5] = analogRead(5);
    light_data[6] = analogRead(6);
    light_data[7] = analogRead(7);

    light_msg.data.data = light_data;
    light_msg.data.size = 8;
    light_msg.data.capacity = 8;
    RCSOFTCHECK(rcl_publish(&light_publisher, &light_msg, NULL));

    sharp_data[0] = analogRead(8);
    sharp_data[1] = analogRead(3);
    sharp_data[2] = analogRead(9);
    sharp_data[3] = analogRead(10);
    sharp_data[4] = analogRead(11);
    sharp_data[5] = analogRead(12);
    sharp_data[6] = analogRead(13);
    sharp_data[7] = analogRead(14);

    sharp_msg.data.data = sharp_data;
    sharp_msg.data.size = 8;
    sharp_msg.data.capacity = 8;

    RCSOFTCHECK(rcl_publish(&sharp_publisher, &sharp_msg, NULL));
    
    line_data[0] = digitalRead(RIGHT_LINE_SENSOR);
    line_data[1] = digitalRead(LEFT_LINE_SENSOR);
    line_data[2] = digitalRead(CENTER_LINE_SENSOR);
    
    line_msg.data.data = line_data;
    line_msg.data.size = 3;
    line_msg.data.capacity = 3;
    RCSOFTCHECK(rcl_publish(&line_publisher, &line_msg, NULL));
    
    encoders_data[0] = encoder_count[LEFT];
    encoders_data[1] = encoder_count[RIGHT];

    encoders_msg.data.data = encoders_data;
    encoders_msg.data.size = 2;
    encoders_msg.data.capacity = 2;
    RCSOFTCHECK(rcl_publish(&enconders_publisher, &encoders_msg, NULL));

    state_data[0] = goal_rpm[LEFT];
    state_data[1] = curr_rpm[LEFT];
    state_data[2] = goal_rpm[RIGHT];
    state_data[3] = curr_rpm[RIGHT];
    state_data[4] = analogRead(1);
    state_data[5] = digitalRead(STOP_BUTTON);
    
    state_msg.data.data = state_data;
    state_msg.data.size = 6;
    state_msg.data.capacity = 6;
    RCSOFTCHECK(rcl_publish(&state_publisher, &state_msg, NULL));
  }
}

void setup() {
  Serial.begin(BAUDRATE);
  set_microros_serial_transports(Serial);
  delay(100);

  float vels_buffer[2];
  vels_msg.data.data = vels_buffer;
  vels_msg.data.size = 0;
  vels_msg.data.capacity = 2;

  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "microcontroller_node", "", &support));

  RCCHECK(rclc_subscription_init_default(&motors_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), "motors_speeds"));

  RCCHECK(rclc_publisher_init_default(&light_publisher,      &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray), "light_sensors"));
  RCCHECK(rclc_publisher_init_default(&sharp_publisher,      &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray), "sharp_sensors"));
  RCCHECK(rclc_publisher_init_default(&line_publisher,       &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray), "line_sensors"));
  RCCHECK(rclc_publisher_init_default(&enconders_publisher,  &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray), "encoders_count"));
  RCCHECK(rclc_publisher_init_default(&state_publisher,      &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray), "current_state"));

  const unsigned int timer_timeout = 1;
  RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(timer_timeout), timer_callback));

  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  RCCHECK(rclc_executor_add_subscription(&executor, &motors_sub, &vels_msg, &subscription_callback, ON_NEW_DATA));

  pinMode(RIGHT_LINE_SENSOR,  INPUT);
  pinMode(LEFT_LINE_SENSOR,   INPUT);
  pinMode(CENTER_LINE_SENSOR, INPUT);
  pinMode(STOP_BUTTON,        INPUT_PULLUP); 

  pinMode(RH_ENCODER_A, INPUT);
  pinMode(RH_ENCODER_B, INPUT);
  pinMode(LH_ENCODER_A, INPUT);
  pinMode(LH_ENCODER_B, INPUT);

  attachInterrupt(digitalPinToInterrupt(LH_ENCODER_A), left_encoder_event,  CHANGE);
  attachInterrupt(digitalPinToInterrupt(RH_ENCODER_A), right_encoder_event, CHANGE);
  
  ledcSetup(PWM_CHANNEL_LEFT1,  PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(PWM_CHANNEL_LEFT2,  PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(PWM_CHANNEL_RIGHT1, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(PWM_CHANNEL_RIGHT2, PWM_FREQ, PWM_RESOLUTION);

  ledcAttachPin(MOTOR_LEFT_INI1,  PWM_CHANNEL_LEFT1);
  ledcAttachPin(MOTOR_LEFT_INI2,  PWM_CHANNEL_LEFT2);
  ledcAttachPin(MOTOR_RIGHT_INI3, PWM_CHANNEL_RIGHT1);
  ledcAttachPin(MOTOR_RIGHT_INI4, PWM_CHANNEL_RIGHT2);
}

void loop() {
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1)));
}
