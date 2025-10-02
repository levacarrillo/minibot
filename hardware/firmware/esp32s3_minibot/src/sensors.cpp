#include "sensors.h"
#include "config.h"

int32_t light_data[8];
int32_t sharp_data[8];
int32_t line_data[3];

void read_light_sensors() {
  for (int i = 0; i < 8; i++) {
    light_data[i] = analogRead(18 - i); // Ajusta al orden real de pines
  }
}

void read_sharp_sensors() {
  for (int i = 0; i < 8; i++) {
    sharp_data[i] = analogRead(8 + i);
  }
}

void read_line_sensors() {
  line_data[0] = digitalRead(RIGHT_LINE_SENSOR);
  line_data[1] = digitalRead(LEFT_LINE_SENSOR);
  line_data[2] = digitalRead(CENTER_LINE_SENSOR);
}
