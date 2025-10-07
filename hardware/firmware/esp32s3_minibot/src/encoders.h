#ifndef ENCODERS_H
#define ENCODERS_H

#include <Arduino.h>


extern volatile long encoder_count[2];
extern volatile long encoder_last_count[2];
void setup_encoders();

#endif
