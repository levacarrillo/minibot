#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// --- DEFINITIONS ---
#define LEFT   0
#define RIGHT  1

// --- COMUNICATION ---
#define BAUDRATE 3000000

// --- DIGITAL SENSOR PINS ---
#define RIGHT_LINE_SENSOR   36
#define LEFT_LINE_SENSOR    37
#define CENTER_LINE_SENSOR  42

// --- ENCODERS PINS ---
#define RH_ENCODER_A  20
#define RH_ENCODER_B  39
#define LH_ENCODER_A  35
#define LH_ENCODER_B  47

// --- PWM ---
#define PWM_FREQ          5000
#define PWM_RESOLUTION       8
#define PWM_CHANNEL_LEFT1    0
#define PWM_CHANNEL_LEFT2    1
#define PWM_CHANNEL_RIGHT1   2
#define PWM_CHANNEL_RIGHT2   3

// --- DIGITAL OUTPUTS ---
#define MOTOR_LEFT_INI1     45
#define MOTOR_LEFT_INI2     41
#define MOTOR_RIGHT_INI3    38
#define MOTOR_RIGHT_INI4    21

// --- ANALOG READINGS ---
#define LDR_SENSOR_1        18
#define LDR_SENSOR_2        17
#define LDR_SENSOR_3        16
#define LDR_SENSOR_4        15
#define LDR_SENSOR_5         4
#define LDR_SENSOR_6         5
#define LDR_SENSOR_7         6
#define LDR_SENSOR_8         7

#define SHARP_SENSOR_1       8
#define SHARP_SENSOR_2       3
#define SHARP_SENSOR_3       9
#define SHARP_SENSOR_4      10
#define SHARP_SENSOR_5      11 
#define SHARP_SENSOR_6      12 
#define SHARP_SENSOR_7      13 
#define SHARP_SENSOR_8      14

#define BATTERY_SENSOR_1     1
#define BATTERY_SENSOR_2     2


// --- CONTROL ---
const int sampling_time = 10; // MILLISECONDS
const int max_rpm = 350;
const unsigned int pulses_per_turn = 600;
const float wheel_diameter = 43.38 / 1000.0;

const float Kp[2] = {2.5, 2.5};
const float Ki[2] = {0.014, 0.012};
const float Kd[2] = {1.5, 3.5};

#endif
