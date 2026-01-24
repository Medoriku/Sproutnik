#include <Arduino.h>

struct AirflowData {
  float velocity_m_s = 0.0f;
};

volatile unsigned long airflow_pulseCount = 0;
uint8_t airflow_pin = 2;
float airflow_kFactor = 100000.0f;
float airflow_area_m2 = 0.0f;
uint32_t airflow_windowSeconds = 10;
unsigned long airflow_lastTick = 0;
unsigned long airflow_pulseAccumulator = 0;
unsigned long airflow_elapsedMs = 0;
AirflowData airflow_data;

void Airflow_countPulse() {
  airflow_pulseCount++;
}

void Airflow_begin(uint8_t pin, float kFactor, float diameter_m, uint32_t windowSeconds) {
  airflow_pin = pin;
  airflow_kFactor = kFactor;
  airflow_windowSeconds = windowSeconds;
  const float radius = diameter_m * 0.5f;
  airflow_area_m2 = 3.14159265f * radius * radius;

  pinMode(airflow_pin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(airflow_pin), Airflow_countPulse, RISING);
  airflow_lastTick = millis();
}

void Airflow_update() {
  const unsigned long now = millis();
  if (now - airflow_lastTick >= 1000UL) {
    noInterrupts();
    unsigned long pulses = airflow_pulseCount;
    airflow_pulseCount = 0;
    interrupts();

    airflow_pulseAccumulator += pulses;
    airflow_elapsedMs += 1000UL;

    if (airflow_elapsedMs >= airflow_windowSeconds * 1000UL) {
      const float liters = airflow_pulseAccumulator / airflow_kFactor;
      const float flow_L_min = liters * (60.0f / airflow_windowSeconds);
      const float flow_m3_s = (flow_L_min / 1000.0f) / 60.0f;
      airflow_data.velocity_m_s = (airflow_area_m2 > 0.0f) ? flow_m3_s / airflow_area_m2 : 0.0f;
      airflow_pulseAccumulator = 0;
      airflow_elapsedMs = 0;
    }

    airflow_lastTick = now;
  }
}

AirflowData Airflow_getData() {
  return airflow_data;
}
