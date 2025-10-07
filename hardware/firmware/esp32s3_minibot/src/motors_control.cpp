#include "motors_control.h"
#include "encoders.h"
#include "config.h"

float P[2] = {4.0, 4.0};
float I[2] = {0.005, 0.005};
float D[2] = {0.0, 0.0};

float integral_error[2] = {0.0, 0.0};
float prev_error[2] = {0.0, 0.0};
float pid_output[2] = {0.0, 0.0};

int curr_rpm[2]   = {0, 0};
int goal_rpm[2]   = {0, 0};
float goal_speed[2] = {0.0, 0.0};

unsigned long current_time  = 0;
unsigned long previous_time = 0;

void setup_motors() {
  ledcSetup(PWM_CHANNEL_LEFT1, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(PWM_CHANNEL_LEFT2, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(PWM_CHANNEL_RIGHT1, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(PWM_CHANNEL_RIGHT2, PWM_FREQ, PWM_RESOLUTION);

  ledcAttachPin(MOTOR_LEFT_INI1, PWM_CHANNEL_LEFT1);
  ledcAttachPin(MOTOR_LEFT_INI2, PWM_CHANNEL_LEFT2);
  ledcAttachPin(MOTOR_RIGHT_INI3, PWM_CHANNEL_RIGHT1);
  ledcAttachPin(MOTOR_RIGHT_INI4, PWM_CHANNEL_RIGHT2);
  previous_time = millis();
}

void set_speeds_reference(float goal_left, float goal_right) {
  goal_speed[LEFT]  = goal_left;
  goal_speed[RIGHT] = goal_right;
  goal_rpm[LEFT]  = fabs((goal_left  * 60) / (M_PI * wheel_diameter));
  goal_rpm[RIGHT] = fabs((goal_right * 60) / (M_PI * wheel_diameter));
  // curr_rpm[LEFT]  = int(goal_left);
  // curr_rpm[RIGHT] = int(goal_right);
}

void calculate_rpms() {
  current_time = millis();
  unsigned long elapsed_time = current_time - previous_time;
  if (elapsed_time >= sampling_time) {
    volatile long delta_left  = fabs(encoder_count[LEFT]  - encoder_last_count[LEFT]);
    volatile long delta_right = fabs(encoder_count[RIGHT] - encoder_last_count[RIGHT]);

    curr_rpm[LEFT]  = 60000 * delta_left  / (pulses_per_turn * elapsed_time);
    curr_rpm[RIGHT] = 60000 * delta_right / (pulses_per_turn * elapsed_time);
    
    encoder_last_count[LEFT]  = encoder_count[LEFT];
    encoder_last_count[RIGHT] = encoder_count[RIGHT];
    previous_time = current_time;
  }

}

int pid_to_pwm(float pid_val, float max_ref) {
  float abs_v = fabs(pid_val);
  if (abs_v > max_ref) abs_v = max_ref;
  return (int)((abs_v / max_ref) * 255.0);
}

void apply_pwm(int side, int pwm, float goal_speed_val) {
  if (fabs(goal_speed_val) < 1e-6) {
    if (side == LEFT) { ledcWrite(PWM_CHANNEL_LEFT1, 0); ledcWrite(PWM_CHANNEL_LEFT2, 0); }
    else              { ledcWrite(PWM_CHANNEL_RIGHT1, 0); ledcWrite(PWM_CHANNEL_RIGHT2, 0); }
  } else {
    if (goal_speed_val < 0) {
      if (side == LEFT) { ledcWrite(PWM_CHANNEL_LEFT1, pwm); ledcWrite(PWM_CHANNEL_LEFT2, 0); }
      else              { ledcWrite(PWM_CHANNEL_RIGHT1, pwm); ledcWrite(PWM_CHANNEL_RIGHT2, 0); }
    } else {
      if (side == LEFT) { ledcWrite(PWM_CHANNEL_LEFT1, 0); ledcWrite(PWM_CHANNEL_LEFT2, pwm); }
      else              { ledcWrite(PWM_CHANNEL_RIGHT1, 0); ledcWrite(PWM_CHANNEL_RIGHT2, pwm); }
    }
  }
}
