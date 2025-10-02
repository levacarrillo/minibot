#ifndef ENCODERS_H
#define ENCODERS_H

#include <Arduino.h>

#define LEFT 0
#define RIGHT 1

extern volatile long encoder_count[2];

void setup_encoders();

#endif
