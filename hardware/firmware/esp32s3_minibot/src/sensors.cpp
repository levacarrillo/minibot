#include "sensors.h"
#include "config.h"


int32_t sensors_data[19];


void read_light_sensors() {
  sensors_data[0] = analogRead(18);
  sensors_data[1] = analogRead(17);
  sensors_data[2] = analogRead(16);
  sensors_data[3] = analogRead(15);
  sensors_data[4] = analogRead(4);
  sensors_data[5] = analogRead(5);
  sensors_data[6] = analogRead(6);
  sensors_data[7] = analogRead(7);
}

void read_sharp_sensors() {
  sensors_data[8] = analogRead(8);
  sensors_data[9] = analogRead(3);
  sensors_data[10] = analogRead(9);
  sensors_data[11] = analogRead(10);
  sensors_data[12] = analogRead(11);
  sensors_data[13] = analogRead(12);
  sensors_data[14] = analogRead(13);
  sensors_data[15] = analogRead(14);
}

void read_line_sensors() {
  sensors_data[16] = digitalRead(RIGHT_LINE_SENSOR);
  sensors_data[17] = digitalRead(LEFT_LINE_SENSOR);
  sensors_data[18] = digitalRead(CENTER_LINE_SENSOR);
}
