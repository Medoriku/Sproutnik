#include <Arduino.h>

struct SoilData {
  float soil1_percent = 0.0f;
  float soil2_percent = 0.0f;
  float soil3_percent = 0.0f;
};

int soil_pin1 = A0;
int soil_pin2 = A1;
int soil_pin3 = A2;
int soil_power1 = -1;
int soil_power2 = -1;
int soil_power3 = -1;
uint32_t soil_intervalMs = 10000;
unsigned long soil_lastMeasurement = 0;
SoilData soil_data;

float Soil_rawToPercent(int rawValue) {
  return (rawValue / 1023.0f) * 100.0f;
}

void Soil_configurePowerPin(int pin) {
  if (pin >= 0) {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, HIGH);
  }
}

void Soil_begin(int pin1, int pin2, int pin3, int power1, int power2, int power3, uint32_t intervalMs) {
  soil_pin1 = pin1;
  soil_pin2 = pin2;
  soil_pin3 = pin3;
  soil_power1 = power1;
  soil_power2 = power2;
  soil_power3 = power3;
  soil_intervalMs = intervalMs;

  pinMode(soil_pin1, INPUT);
  pinMode(soil_pin2, INPUT);
  pinMode(soil_pin3, INPUT);

  Soil_configurePowerPin(soil_power1);
  Soil_configurePowerPin(soil_power2);
  Soil_configurePowerPin(soil_power3);

  soil_lastMeasurement = millis();
}

void Soil_update() {
  const unsigned long now = millis();
  if (now - soil_lastMeasurement >= soil_intervalMs) {
    soil_lastMeasurement = now;

    const int soil1_raw = analogRead(soil_pin1);
    const int soil2_raw = analogRead(soil_pin2);
    const int soil3_raw = analogRead(soil_pin3);

    soil_data.soil1_percent = Soil_rawToPercent(soil1_raw);
    soil_data.soil2_percent = Soil_rawToPercent(soil2_raw);
    soil_data.soil3_percent = Soil_rawToPercent(soil3_raw);
  }
}

SoilData Soil_getData() {
  return soil_data;
}
