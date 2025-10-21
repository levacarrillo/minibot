#ifndef SENSORS_H
#define SENSORS_H

#include <Arduino.h>
#include <driver/temp_sensor.h>

extern int32_t sensors_data[23];

void read_sensors();

#endif
