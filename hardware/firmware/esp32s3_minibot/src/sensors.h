#ifndef SENSORS_H
#define SENSORS_H

#include <Arduino.h>

extern int32_t light_data[8];
extern int32_t sharp_data[8];
extern int32_t line_data[3];

void read_light_sensors();
void read_sharp_sensors();
void read_line_sensors();

#endif
