#include "motor_control.h"
#include "config.h"

float P[2] = {4.0, 4.0};
float I[2] = {0.005, 0.005};
float D[2] = {0.0, 0.0};

float integral_error[2] = {0.0,0.0};
float prev_error[2] = {0.0,0.0};
float pid_output[2] = {0.0,0.0};

float goal_speed[2] = {0.0,0.0};
float goal_rpm[2]   = {0.0,0.0};
float curr_rpm[2]   = {0.0,0.0};

void setup_motors() {
  ledcSetup(PWM_CHANNEL_LEFT1, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(PWM_CHANNEL_LEFT2, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(PWM_CHANNEL_RIGHT1, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(PWM_CHANNEL_RIGHT2, PWM_FREQ, PWM_RESOLUTION);

  ledcAttachPin(MOTOR_LEFT_INI1, PWM_CHANNEL_LEFT1);
  ledcAttachPin(MOTOR_LEFT_INI2, PWM_CHANNEL_LEFT2);
  ledcAttachPin(MOTOR_RIGHT_INI3, PWM_CHANNEL_RIGHT1);
  ledcAttachPin(MOTOR_RIGHT_INI4, PWM_CHANNEL_RIGHT2);
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
