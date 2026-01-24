#include <Wire.h>
#include "SparkFunBME280.h"

struct BmeData {
  bool ok = false;
  float temp_c = 0.0f;
  float humidity = 0.0f;
  float pressure_kpa = 0.0f;
  float altitude_m = 0.0f;
};

BME280 bme_sensor;
BmeData bme_data;
uint8_t bme_primary = 0x77;
uint8_t bme_alternate = 0x76;

bool BME_begin(TwoWire &wire = Wire, uint8_t primary = 0x77, uint8_t alternate = 0x76) {
  bme_primary = primary;
  bme_alternate = alternate;

  bme_sensor.setI2CAddress(bme_primary);
  if (bme_sensor.beginI2C(wire)) {
    bme_data.ok = true;
  } else {
    bme_sensor.setI2CAddress(bme_alternate);
    bme_data.ok = bme_sensor.beginI2C(wire);
  }

  if (bme_data.ok) {
    bme_sensor.setTempOverSample(4);
    bme_sensor.setPressureOverSample(4);
    bme_sensor.setHumidityOverSample(4);
  }

  return bme_data.ok;
}

void BME_update() {
  if (!bme_data.ok) return;
  bme_data.temp_c = bme_sensor.readTempC();
  bme_data.humidity = bme_sensor.readFloatHumidity();
  bme_data.pressure_kpa = bme_sensor.readFloatPressure() / 1000.0f;
  bme_data.altitude_m = bme_sensor.readFloatAltitudeMeters();
}

BmeData BME_getData() {
  return bme_data;
}
