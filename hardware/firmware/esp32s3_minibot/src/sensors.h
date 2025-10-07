#ifndef SENSORS_H
#define SENSORS_H

#include <Arduino.h>

extern int32_t sensors_data[19];

void read_light_sensors();
void read_sharp_sensors();
void read_line_sensors();

#endif
