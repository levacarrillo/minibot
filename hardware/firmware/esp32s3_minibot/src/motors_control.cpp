#include "motors_control.h"
#include "encoders.h"
#include "config.h"

float Kp[2] = {1.8, 1.8};
float Ki[2] = {0.005, 0.005};
float Kd[2] = {0.001, 0.001};

float error_rpm[2] = {0.0, 0.0};
float integral_error[2] = {0.0, 0.0};
float prev_error[2] = {0.0, 0.0};

float pid_output[2] = {0.0, 0.0};


int curr_rpm[2]   = {0, 0};
int goal_rpm[2]   = {0, 0};
float goal_speed[2] = {0.0, 0.0};

int pwm[2] = {0, 0};

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
}

void calculate_rpms() {
  current_time = millis();
  unsigned long elapsed_time = current_time - previous_time;
  if (elapsed_time >= sampling_time) {
    volatile long delta_left  = fabs(encoders_count[LEFT]  - encoders_last_count[LEFT]);
    volatile long delta_right = fabs(encoders_count[RIGHT] - encoders_last_count[RIGHT]);

    curr_rpm[LEFT]  = 60000 * delta_left  / (pulses_per_turn * elapsed_time);
    curr_rpm[RIGHT] = 60000 * delta_right / (pulses_per_turn * elapsed_time);
    
    encoders_last_count[LEFT]  = encoders_count[LEFT];
    encoders_last_count[RIGHT] = encoders_count[RIGHT];
    
    error_rpm[LEFT]  = goal_rpm[LEFT]  - curr_rpm[LEFT];
    error_rpm[RIGHT] = goal_rpm[RIGHT] - curr_rpm[RIGHT];

    integral_error[LEFT]  += error_rpm[LEFT]  * elapsed_time;
    integral_error[RIGHT] += error_rpm[RIGHT] * elapsed_time;

    
    previous_time = current_time;
    compute_pid();
    prev_error[LEFT]  = error_rpm[LEFT];
    prev_error[RIGHT] = error_rpm[RIGHT];
  }
}

void compute_pid() {
  float derivative_left  = (error_rpm[LEFT]  - prev_error[LEFT])  / sampling_time;
  float derivative_right = (error_rpm[RIGHT] - prev_error[RIGHT]) / sampling_time;

  pid_output[LEFT]  = Kp[LEFT]  * error_rpm[LEFT]  + Ki[LEFT]  * integral_error[LEFT]  + Kd[LEFT]  * derivative_left;
  pid_output[RIGHT] = Kp[RIGHT] * error_rpm[RIGHT] + Ki[RIGHT] * integral_error[RIGHT] + Kd[RIGHT] * derivative_right;
  
  pid_to_pwm();
}

void pid_to_pwm() {
  pwm[LEFT]  = constrain(map(pid_output[LEFT],  0, max_rpm, 0, 255), 0, 255);
  pwm[RIGHT] = constrain(map(pid_output[RIGHT], 0, max_rpm, 0, 255), 0, 255);

  if (fabs(goal_speed[LEFT]) < 1e-6) { ledcWrite(PWM_CHANNEL_LEFT1, 0); ledcWrite(PWM_CHANNEL_LEFT2, 0); }
  else { 
    if (goal_speed[LEFT] < 0) { ledcWrite(PWM_CHANNEL_LEFT1, pwm[LEFT]); ledcWrite(PWM_CHANNEL_LEFT2, 0); }
    else { ledcWrite(PWM_CHANNEL_LEFT1, 0); ledcWrite(PWM_CHANNEL_LEFT2, pwm[LEFT]); }
  }

  if (fabs(goal_speed[RIGHT]) < 1e-6) { ledcWrite(PWM_CHANNEL_RIGHT1, 0); ledcWrite(PWM_CHANNEL_RIGHT2, 0); }
  else {
    if (goal_speed[RIGHT] < 0) { ledcWrite(PWM_CHANNEL_RIGHT1, pwm[RIGHT]); ledcWrite(PWM_CHANNEL_RIGHT2, 0); }
    else { ledcWrite(PWM_CHANNEL_RIGHT1, 0); ledcWrite(PWM_CHANNEL_RIGHT2, pwm[RIGHT]); }
  }  
}
