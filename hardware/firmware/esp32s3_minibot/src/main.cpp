#include <Arduino.h>
#include "config.h"
#include "encoders.h"
#include "motors_control.h"
#include "sensors.h"
#include "ros_setup.h"

void setup() {
  Serial.begin(BAUDRATE);
  setup_encoders();
  setup_motors();
  setup_ros();
}

void loop() {
  loop_ros();
}
