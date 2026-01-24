#include <Wire.h>
#include "pas-co2-ino.hpp"

struct Co2Data {
  bool ok = false;
  int16_t co2_ppm = 0;
};

PASCO2Ino co2_sensor(&Wire);
Co2Data co2_data;

void CO2_begin(TwoWire &wire = Wire) {
  if (co2_sensor.begin() == XENSIV_PASCO2_OK) {
    co2_data.ok = true;
    if (co2_sensor.startMeasure(10) != XENSIV_PASCO2_OK) {
      co2_data.ok = false;
    }
  } else {
    co2_data.ok = false;
  }
}

void CO2_update() {
  if (!co2_data.ok) return;
  int16_t co2ppm = 0;
  if (co2_sensor.getCO2(co2ppm) == XENSIV_PASCO2_OK && co2ppm > 0) {
    co2_data.co2_ppm = co2ppm;
  }
}

Co2Data CO2_getData() {
  return co2_data;
}
