/*
 * Environmental Monitoring System - Final Version
 * 
 * Author: Diyora Daminova
 * Created: January 21, 2026
 * Last Updated: Feb 12, 2026
 * 
 * Description:
 *   Multi-sensor environmental monitoring system for Arduino GIGA.
 *   Uses dual I2C buses:
 *     - Wire  : all sensors except CO₂
 *     - Wire1 : PASCO2 CO₂ sensor (connected to SDA1/SCL1)
 */

// ============================================================================
// LIBRARY INCLUDES
// ============================================================================
#include <Arduino.h>
#include "DFRobot_OxygenSensor.h"
#include <Wire.h>
#include "SparkFunBME280.h"
#include <SparkFun_TMP117.h>
#include "pas-co2-ino.hpp"
#include <Adafruit_Sensor.h>
#include <Adafruit_TSL2591.h>

// ============================================================================
// CONSTANTS
// ============================================================================
#define I2C_FREQ_HZ 400000
#define PRESSURE_REFERENCE 900
#define SOIL_MEAS_INTERVAL_MS 10000
#define PERIODIC_MEAS_INTERVAL_IN_SECONDS 60L

// ============================================================================
// SOIL MOISTURE SENSOR CONFIGURATION
// ============================================================================
#define SOIL_1_PIN A0
#define SOIL_2_PIN A1
#define SOIL_3_PIN A2
#define SOIL_1_PWR_PIN -1
#define SOIL_2_PWR_PIN -1
#define SOIL_3_PWR_PIN -1

unsigned long soilLastMeasurement = 0;
float latest_soil1_percent = 0.0;
float latest_soil2_percent = 0.0;
float latest_soil3_percent = 0.0;

// ============================================================================
// BME280 ATMOSPHERIC SENSOR
// ============================================================================
BME280 bme;
bool bme_ok = false;
float latest_bme_temp_c = 0.0;
float latest_bme_humidity = 0.0;
float latest_bme_pressure_kpa = 0.0;
float latest_bme_altitude_m = 0.0;

// ============================================================================
// TMP117 TEMPERATURE SENSORS
// ============================================================================
TMP117 tmp117_1;  // 0x48
TMP117 tmp117_2;  // 0x4A
bool tmp117_1_ok = false;
bool tmp117_2_ok = false;
float latest_tmp117_1_c = 0.0;
float latest_tmp117_2_c = 0.0;

// ============================================================================
// CO2 SENSOR (PASCO2 on Wire1)
// ============================================================================
PASCO2Ino cotwo(&Wire1);
int16_t co2ppm = 0;
Error_t co2_err;

// ============================================================================
// LIGHT SENSOR (TSL2591)
// ============================================================================
Adafruit_TSL2591 tsl = Adafruit_TSL2591(2591);
bool light_ok = false;
float latest_light_lux = 0.0;

// ============================================================================
// OXYGEN SENSOR (DFRobot SEN0322)
// ============================================================================
#define Oxygen_IICAddress ADDRESS_3
#define COLLECT_NUMBER 10
#define O2_MEAS_INTERVAL_MS 1000

DFRobot_OxygenSensor oxygen;
unsigned long o2LastMeasurement = 0;
float latest_o2_percent = 0.0;

// ============================================================================
// GLOBAL PRINT SCHEDULER
// ============================================================================
const unsigned long PRINT_INTERVAL_MS = 60000;
unsigned long lastCombinedPrint = 0;

// ============================================================================
// SETUP
// ============================================================================
void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); }
  Serial.println("System initializing...");

  // --- Initialize I2C buses ---
  Wire.begin();         // main I2C bus
  Wire.setClock(I2C_FREQ_HZ);
  Wire1.begin();        // secondary I2C bus (CO2)
  Wire1.setClock(I2C_FREQ_HZ);

  // --- Initialize PASCO2 (on Wire1) ---
  co2_err = cotwo.begin();
  if (co2_err != XENSIV_PASCO2_OK) {
    Serial.print("PASCO2 init error: "); Serial.println(co2_err);
  }
  co2_err = cotwo.setPressRef(PRESSURE_REFERENCE);
  co2_err = cotwo.startMeasure(PERIODIC_MEAS_INTERVAL_IN_SECONDS);
  delay(1000);

  // --- Initialize soil sensors ---
  pinMode(SOIL_1_PIN, INPUT);
  pinMode(SOIL_2_PIN, INPUT);
  pinMode(SOIL_3_PIN, INPUT);
  if (SOIL_1_PWR_PIN >= 0) { pinMode(SOIL_1_PWR_PIN, OUTPUT); digitalWrite(SOIL_1_PWR_PIN, HIGH); }
  if (SOIL_2_PWR_PIN >= 0) { pinMode(SOIL_2_PWR_PIN, OUTPUT); digitalWrite(SOIL_2_PWR_PIN, HIGH); }
  if (SOIL_3_PWR_PIN >= 0) { pinMode(SOIL_3_PWR_PIN, OUTPUT); digitalWrite(SOIL_3_PWR_PIN, HIGH); }

  // --- Initialize BME280 ---
  bme.setI2CAddress(0x77);
  if (bme.beginI2C(Wire)) bme_ok = true;
  else { bme.setI2CAddress(0x76); bme_ok = bme.beginI2C(Wire); }
  if (bme_ok) {
    bme.setTempOverSample(4);
    bme.setPressureOverSample(4);
    bme.setHumidityOverSample(4);
  }

  // --- Initialize TMP117 sensors ---
  tmp117_1_ok = tmp117_1.begin(0x48, Wire);
  tmp117_2_ok = tmp117_2.begin(0x4A, Wire);

  // --- Initialize Oxygen sensor ---
  while (!oxygen.begin(Oxygen_IICAddress)) { delay(1000); }

  // --- Initialize Light sensor ---
  if (tsl.begin()) {
    light_ok = true;
    tsl.setGain(TSL2591_GAIN_MED);
    tsl.setTiming(TSL2591_INTEGRATIONTIME_300MS);
  }

  Serial.println("All sensors initialized.\n");
  Serial.println("Soil1_%  Soil2_%  Soil3_%  TMP1_C  TMP2_C  BME_T_C  BME_RH_%  BME_P_kPa  BME_Alt_m  O2_%  CO2_ppm  Light_Lux");
}

// ============================================================================
// LOOP
// ============================================================================
void loop() {
  unsigned long now = millis();

  // --- BME280 update ---
  static unsigned long bmeLastUpdate = 0;
  if (now - bmeLastUpdate >= 1000 && bme_ok) {
    bmeLastUpdate = now;
    latest_bme_temp_c = bme.readTempC();
    latest_bme_humidity = bme.readFloatHumidity();
    latest_bme_pressure_kpa = bme.readFloatPressure() / 1000.0f;
    latest_bme_altitude_m = bme.readFloatAltitudeMeters();
  }

  // --- PASCO2 update (Wire1) ---
  static unsigned long co2LastReadMs = 0;
  if (now - co2LastReadMs >= PERIODIC_MEAS_INTERVAL_IN_SECONDS * 1000UL) {
    co2LastReadMs = now;
    co2_err = cotwo.getCO2(co2ppm);
    if (co2_err != XENSIV_PASCO2_OK) {
      if (co2_err == XENSIV_PASCO2_ERR_COMM) {
        delay(600);
        co2_err = cotwo.getCO2(co2ppm);
      }
      Serial.print("CO₂ read error: "); Serial.println(co2_err);
    }
    co2_err = cotwo.setPressRef(PRESSURE_REFERENCE);
  }

  // --- TMP117 update ---
  if (tmp117_1_ok && tmp117_1.dataReady()) latest_tmp117_1_c = tmp117_1.readTempC();
  if (tmp117_2_ok && tmp117_2.dataReady()) latest_tmp117_2_c = tmp117_2.readTempC();

  // --- Light sensor update ---
  if (light_ok) {
    uint32_t lum = tsl.getFullLuminosity();
    uint16_t ir = lum >> 16;
    uint16_t full = lum & 0xFFFF;
    latest_light_lux = tsl.calculateLux(full, ir);
  }

  // --- Soil moisture update ---
  if (now - soilLastMeasurement >= SOIL_MEAS_INTERVAL_MS) {
    soilLastMeasurement = now;
    int s1 = analogRead(SOIL_1_PIN);
    int s2 = analogRead(SOIL_2_PIN);
    int s3 = analogRead(SOIL_3_PIN);
    latest_soil1_percent = (s1 / 1023.0) * 100.0;
    latest_soil2_percent = (s2 / 1023.0) * 100.0;
    latest_soil3_percent = (s3 / 1023.0) * 100.0;
  }

  // --- O2 update ---
  if (now - o2LastMeasurement >= O2_MEAS_INTERVAL_MS) {
    o2LastMeasurement = now;
    latest_o2_percent = oxygen.getOxygenData(COLLECT_NUMBER);
  }

  // --- Print data every 60 sec ---
  if (now - lastCombinedPrint >= PRINT_INTERVAL_MS) {
    lastCombinedPrint = now;
    Serial.print(latest_soil1_percent, 2); Serial.print(", ");
    Serial.print(latest_soil2_percent, 2); Serial.print(", ");
    Serial.print(latest_soil3_percent, 2); Serial.print(", ");
    Serial.print(latest_tmp117_1_c, 2);    Serial.print(", ");
    Serial.print(latest_tmp117_2_c, 2);    Serial.print(", ");
    Serial.print(latest_bme_temp_c, 2);    Serial.print(", ");
    Serial.print(latest_bme_humidity, 1);  Serial.print(", ");
    Serial.print(latest_bme_pressure_kpa, 3); Serial.print(", ");
    Serial.print(latest_bme_altitude_m, 1);  Serial.print(", ");
    Serial.print(latest_o2_percent, 2);    Serial.print(", ");
    Serial.print(co2ppm);                  Serial.print(", ");
    Serial.println(latest_light_lux, 2);
  }

  delay(10);
}