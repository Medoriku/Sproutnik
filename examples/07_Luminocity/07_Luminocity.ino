#include <Adafruit_Sensor.h>
#include <Adafruit_TSL2591.h>

struct LightData {
  bool ok = false;
  uint16_t ir = 0;
  uint16_t full = 0;
  uint16_t visible = 0;
  float lux = 0.0f;
};

Adafruit_TSL2591 light_sensor = Adafruit_TSL2591(2591);
LightData light_data;

void Light_begin() {
  light_data.ok = light_sensor.begin();
  if (light_data.ok) {
    light_sensor.setGain(TSL2591_GAIN_MED);
    light_sensor.setTiming(TSL2591_INTEGRATIONTIME_300MS);
  }
}

void Light_update() {
  if (!light_data.ok) return;
  const uint32_t lum = light_sensor.getFullLuminosity();
  light_data.ir = lum >> 16;
  light_data.full = lum & 0xFFFF;
  light_data.visible = light_data.full - light_data.ir;
  light_data.lux = light_sensor.calculateLux(light_data.full, light_data.ir);
}

LightData Light_getData() {
  return light_data;
}
