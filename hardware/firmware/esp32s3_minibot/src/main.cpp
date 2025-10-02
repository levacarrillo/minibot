#include <Arduino.h>
#include "config.h"
#include "encoders.h"
#include "motor_control.h"
#include "sensors.h"
#include "ros_setup.h"

void setup() {
  Serial.begin(BAUDRATE);
  setup_encoders();
  setup_motors();
  setup_ros();   // inicializa ROS, publishers/subscribers
}

void loop() {
  loop_ros();    // ejecuta callbacks y timer
}
