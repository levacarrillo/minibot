#include "alarm.h"
#include "config.h"

void setup_alarm() {
    pinMode(ALARM_BUZZER, OUTPUT);
}

void check_battery_levels(int battery_1, int battery_2){
    if (battery_1 < BATTERY_THRESHOLD || battery_2 < BATTERY_THRESHOLD) {
        tone(ALARM_BUZZER, PWM_FREQ_ALARM);
    } else {
        tone(ALARM_BUZZER, 0);
    }
}