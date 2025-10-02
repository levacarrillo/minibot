#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// --- COMUNICATION ---
#define BAUDRATE 3000000

// --- DIGITAL SENSOR PINS ---
#define STOP_BUTTON 39
#define RIGHT_LINE_SENSOR 2
#define LEFT_LINE_SENSOR 37
#define CENTER_LINE_SENSOR 42

#define LEFT 0
#define RIGHT 1

// --- ENCODERS PINS ---
#define RH_ENCODER_A 20
#define RH_ENCODER_B 19
#define LH_ENCODER_A 35
#define LH_ENCODER_B 47

// --- PWM ---
#define PWM_FREQ 5000
#define PWM_RESOLUTION 8
#define PWM_CHANNEL_LEFT1 0
#define PWM_CHANNEL_LEFT2 1
#define PWM_CHANNEL_RIGHT1 2
#define PWM_CHANNEL_RIGHT2 3

#define MOTOR_LEFT_INI1 40
#define MOTOR_LEFT_INI2 41
#define MOTOR_RIGHT_INI3 38
#define MOTOR_RIGHT_INI4 21

// --- CONTROL ---
const unsigned int pulses_per_turn = 600;
const int max_rpm = 350;
const float wheel_diameter = 43.38 / 1000.0;

#endif
