#ifndef MOTORS_CONTROL_H
#define MOTORS_CONTROL_H

#include <Arduino.h>

extern float goal_speed[2];
extern int goal_rpm[2], curr_rpm[2];

void setup_motors();
void set_speeds_reference(float goal_left, float goal_right);

void calculate_rpms();
void compute_pid();
void pid_to_pwm();

#endif
