#ifndef MOTORS_CONTROL_H
#define MOTORS_CONTROL_H

#include <Arduino.h>

extern float goal_speed[2];
extern int goal_rpm[2], curr_rpm[2];

void setup_motors();
void calculate_rpms();
void set_speeds_reference(float goal_left, float goal_right);
int  pid_to_pwm(float pid_val, float max_ref);
void apply_pwm(int side, int pwm, float goal_speed_val);

#endif
