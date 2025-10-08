#include "encoders.h"
#include "config.h"

volatile long encoders_count[2] = {0, 0};
volatile long encoders_last_count[2] = {0, 0};

void IRAM_ATTR left_encoder_event() {
  bool A = digitalRead(LH_ENCODER_A);
  bool B = digitalRead(LH_ENCODER_B);
  if (A == B) encoders_count[LEFT]++;
  else encoders_count[LEFT]--;
}

void IRAM_ATTR right_encoder_event() {
  bool A = digitalRead(RH_ENCODER_A);
  bool B = digitalRead(RH_ENCODER_B);
  if (A == B) encoders_count[RIGHT]--;
  else encoders_count[RIGHT]++;
}

void setup_encoders() {
  pinMode(RH_ENCODER_A, INPUT);
  pinMode(RH_ENCODER_B, INPUT);
  pinMode(LH_ENCODER_A, INPUT);
  pinMode(LH_ENCODER_B, INPUT);

  attachInterrupt(digitalPinToInterrupt(LH_ENCODER_A), left_encoder_event, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RH_ENCODER_A), right_encoder_event, CHANGE);
}
