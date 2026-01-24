#include <Arduino.h>

struct O2Data {
  bool warmed_up = false;
  float o2_percent = 0.0f;
};

uint8_t o2_pin = A3;
uint32_t o2_intervalMs = 2000;
uint32_t o2_warmupMs = 300000;
float o2_adcRefVoltage = 3.3f;
int o2_adcMax = 1023;
unsigned long o2_lastMeasurement = 0;
unsigned long o2_startupMs = 0;
O2Data o2_data;

float O2_voltageAt0Percent() { return 1.5f; }
float O2_voltageAt25Percent() { return 0.0f; }
float O2_percentMax() { return 25.0f; }

void O2_begin(uint8_t pin, uint32_t intervalMs, uint32_t warmupMs, float adcRefVoltage, int adcMaxValue) {
  o2_pin = pin;
  o2_intervalMs = intervalMs;
  o2_warmupMs = warmupMs;
  o2_adcRefVoltage = adcRefVoltage;
  o2_adcMax = adcMaxValue;
  pinMode(o2_pin, INPUT);
  o2_startupMs = millis();
  o2_lastMeasurement = 0;
  o2_data.warmed_up = false;
}

void O2_update() {
  const unsigned long now = millis();
  if (!o2_data.warmed_up && (now - o2_startupMs >= o2_warmupMs)) {
    o2_data.warmed_up = true;
  }
  if (!o2_data.warmed_up) return;
  if (now - o2_lastMeasurement < o2_intervalMs) return;

  o2_lastMeasurement = now;
  const int raw = analogRead(o2_pin);
  const float voltage = (raw / static_cast<float>(o2_adcMax)) * o2_adcRefVoltage;
  const float span = (O2_voltageAt0Percent() - O2_voltageAt25Percent());
  float pct = ((O2_voltageAt0Percent() - voltage) / span) * O2_percentMax();
  if (pct < 0.0f) pct = 0.0f;
  if (pct > O2_percentMax()) pct = O2_percentMax();
  o2_data.o2_percent = pct;
}

O2Data O2_getData() {
  return o2_data;
}
