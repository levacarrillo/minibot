#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <Arduino.h>

extern float goal_speed[2], goal_rpm[2], curr_rpm[2];

void setup_motors();
int pid_to_pwm(float pid_val, float max_ref);
void apply_pwm(int side, int pwm, float goal_speed_val);

#endif
