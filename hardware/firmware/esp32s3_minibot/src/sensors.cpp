#include "sensors.h"
#include "config.h"


int32_t sensors_data[22];
temp_sensor_config_t temp_sensor = TSENS_CONFIG_DEFAULT();
float temperature;

void read_sensors() {
  // --- LIGHT SENSORS ---
  sensors_data[0] = analogRead(LDR_SENSOR_1);
  sensors_data[1] = analogRead(LDR_SENSOR_2);
  sensors_data[2] = analogRead(LDR_SENSOR_3);
  sensors_data[3] = analogRead(LDR_SENSOR_4);
  sensors_data[4] = analogRead(LDR_SENSOR_5);
  sensors_data[5] = analogRead(LDR_SENSOR_6);
  sensors_data[6] = analogRead(LDR_SENSOR_7);
  sensors_data[7] = analogRead(LDR_SENSOR_8);
  
  // --- SHARP SENSORS ---
  sensors_data[8]  = analogRead(SHARP_SENSOR_1);
  sensors_data[9]  = analogRead(SHARP_SENSOR_2);
  sensors_data[10] = analogRead(SHARP_SENSOR_3);
  sensors_data[11] = analogRead(SHARP_SENSOR_4);
  sensors_data[12] = analogRead(SHARP_SENSOR_5);
  sensors_data[13] = analogRead(SHARP_SENSOR_6);
  sensors_data[14] = analogRead(SHARP_SENSOR_7);
  sensors_data[15] = analogRead(SHARP_SENSOR_8);
  
  // --- LINE SENSORS ---
  sensors_data[16] = digitalRead(RIGHT_LINE_SENSOR);
  sensors_data[17] = digitalRead(LEFT_LINE_SENSOR);
  sensors_data[18] = digitalRead(CENTER_LINE_SENSOR);
  
  // --- BATTERY SENSORS ---
  sensors_data[19] = analogRead(BATTERY_SENSOR_1);
  sensors_data[20] = analogRead(BATTERY_SENSOR_2);
  
  // --- TEMPERATURE SENSOR ---
  temp_sensor_set_config(temp_sensor);
  temp_sensor_start();
  temp_sensor_read_celsius(&temperature);
  sensors_data[21] = (int)round(temperature);
}
