#include "alarm.h"
#include "config.h"

void setup_alarm() {
    pinMode(ALARM_LED, OUTPUT);
    pinMode(ALARM_BUZZER, OUTPUT);
}

void check_battery_levels(int battery_1, int battery_2){
    if (battery_1 < BATTERY_THRESHOLD || battery_2 < BATTERY_THRESHOLD) {
        digitalWrite(ALARM_LED, HIGH);
        digitalWrite(ALARM_BUZZER, HIGH);
    } else {
        digitalWrite(ALARM_LED, LOW);
        digitalWrite(ALARM_BUZZER, LOW);
    }
}