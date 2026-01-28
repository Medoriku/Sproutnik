/*
 * Environmental Monitoring System - Final Version
 * 
 * Author: Diyora Daminova
 * Created: January 21, 2026
 * Last Updated: January 21, 2026
 * 
 * Description:
 *   Multi-sensor environmental monitoring system for Arduino GIGA.
 *   Integrates airflow, soil moisture, atmospheric, and oxygen sensors
 *   for comprehensive environmental data collection.
 * 
 * Hardware Components:
 *   - Airflow sensor (Digital pulse counter)
 *   - 3x Soil moisture sensors (Analog)
 *   - BME280 atmospheric sensor (I2C)
 *   - Grove O2 sensor (Analog)
 *   - TMP117 x2 (I2C)
 *   - Luminosity sensor (I2C)
 *   - XENSIV PAS CO₂ concentration sensor (I2C) 
 * Updates Log:
 *   - January 21, 2026: Initial release
 */

// ============================================================================
// LIBRARY INCLUDES
// ============================================================================
#include "Oxygen.h"
#include <Wire.h>
#include "SparkFunBME280.h"
#include <SparkFun_TMP117.h>
#include "pas-co2-ino.hpp"
#include <Adafruit_Sensor.h>
#include <Adafruit_TSL2591.h>


// ============================================================================
// AIRFLOW SENSOR CONFIGURATION
// ============================================================================
/*
 * Hardware Setup:
 *   - Red wire:    5V
 *   - Black wire:  GND
 *   - Yellow wire: Digital pin 2 (interrupt)
 * 
 * Calibration:
 *   - K-factor: 100,000 pulses/liter (from datasheet)
 *   - Pipe diameter: 4.6 mm (0.0046 m)
 *   - Measurement window: 10 seconds (smoothing)
 */
volatile unsigned long pulseCount = 0;
const int flowPin = 2;
const float K_FACTOR = 100000.0;
const float DIAMETER = 0.0046;
const float AREA = 3.14159265 * (DIAMETER / 2.0) * (DIAMETER / 2.0);
unsigned long lastTime = 0;
const unsigned long windowSec = 10;
unsigned long pulseAccumulator = 0;
unsigned long elapsedTime = 0;

// ============================================================================
// SOIL MOISTURE SENSOR CONFIGURATION
// ============================================================================
/*
 * Hardware Setup (x3 sensors):
 *   - Red wire:    3.3V
 *   - Black wire:  GND
 *   - Signal wire: Analog pins A0, A1, A2
 * 
 * Note: Power control pins set to -1 (sensors powered directly from 3.3V)
 * Measurement interval: 10 seconds (synchronized with airflow)
 */
#define SOIL_1_PIN A0
#define SOIL_2_PIN A1
#define SOIL_3_PIN A2
#define SOIL_1_PWR_PIN -1
#define SOIL_2_PWR_PIN -1
#define SOIL_3_PWR_PIN -1
#define SOIL_MEAS_INTERVAL_MS 10000

// State variables
unsigned long soilLastMeasurement = 0;

// Latest sensor readings
float latest_soil1_percent = 0.0;
float latest_soil2_percent = 0.0;
float latest_soil3_percent = 0.0;
float latest_velocity_m_s = 0.0;

// ============================================================================
// BME280 ATMOSPHERIC SENSOR CONFIGURATION
// ============================================================================
/*
 * I2C Interface:
 *   - SDA: Board default SDA pin
 *   - SCL: Board default SCL pin
 *   - Addresses: 0x77 (primary) or 0x76 (alternate)
 * 
 * Measurements:
 *   - Temperature (°C)
 *   - Humidity (% RH)
 *   - Pressure (kPa)
 *   - Altitude (m)
 */
BME280 bme;
bool bme_ok = false;
// Latest BME280 readings
float latest_bme_temp_c = 0.0;
float latest_bme_humidity = 0.0;
float latest_bme_pressure_kpa = 0.0;
float latest_bme_altitude_m = 0.0;

// ============================================================================
// TMP117 TEMPERATURE SENSORS
// ============================================================================
/*
 * Dual TMP117 high-accuracy temperature sensors on I2C.
 * Addresses used: 0x48 and 0x4A
 */
TMP117 tmp117_1;  // 0x48
TMP117 tmp117_2;  // 0x4A
bool tmp117_1_ok = false;
bool tmp117_2_ok = false;
float latest_tmp117_1_c = 0.0;
float latest_tmp117_2_c = 0.0;

// ============================================================================
// CO2 SENSOR CONFIGURATION
// ============================================================================
/*
 * SparkFun/Infineon XENSIV PAS CO2 (PASCO2) sensor over I2C.
 * Address: 0x28 (default)
 */
PASCO2Ino co2(&Wire);
bool co2_ok = false;
int16_t latest_co2_ppm = 0;

// ============================================================================
// LIGHT SENSOR CONFIGURATION
// ============================================================================
/*
 * Adafruit TSL2591 High Dynamic Range Digital Light Sensor (I2C).
 * Address: 0x29 (fixed)
 */
Adafruit_TSL2591 tsl = Adafruit_TSL2591(2591);
bool light_ok = false;
uint16_t latest_light_ir = 0;
uint16_t latest_light_full = 0;
uint16_t latest_light_visible = 0;
float latest_light_lux = 0.0;

// OXYGEN SENSOR CONFIGURATION
// ============================================================================
/*
 * Hardware Setup (Grove O2 Sensor Pro - GGC2330-O2):
 *   - Red wire (VCC):   5V
 *   - Black wire (GND): GND
 *   - Green wire (V0):  Analog pin A3
 * 
 * IMPORTANT: Requires 5-minute warmup for accurate readings!
 * 
 * Specifications:
 *   - Range: 0-25% O2
 *   - Calibration: Inverse relationship (higher voltage = lower O2%)
 *   - At 0% O2:  V0 ≈ 1.5V
 *   - At 25% O2: V0 ≈ 0V
 *   - ADC reference: 3.3V (Arduino GIGA)
 */
#define O2_PIN A3
#define O2_MEAS_INTERVAL_MS 2000
#define WARMUP_TIME_MS 300000
#define ADC_REF_VOLTAGE 3.3
#define ADC_MAX_VALUE 1023

// Calibration constants
const float O2_VOLTAGE_AT_0_PERCENT = 1.5;
const float O2_VOLTAGE_AT_25_PERCENT = 0.0;
const float O2_PERCENT_MAX = 25.0;

// State variables
unsigned long o2LastMeasurement = 0;
unsigned long startupTime = 0;
bool warmedUp = false;
float latest_o2_percent = 0.0;

// ============================================================================
// GLOBAL PRINT SCHEDULER
// ============================================================================
const unsigned long PRINT_INTERVAL_MS = 10000; // unified output every 10 seconds
unsigned long lastCombinedPrint = 0;
// ============================================================================
// SETUP FUNCTION
// ============================================================================
/*
 * Initializes all sensors and communication interfaces.
 * Called once at startup.
 */
void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }

  // Initialize airflow sensor
  pinMode(flowPin, INPUT_PULLUP);
  attachInterrupt(
    digitalPinToInterrupt(flowPin),
    countPulse,
    RISING);
  lastTime = millis();
  startupTime = millis();

  // Print system information
  Serial.println("=== ENVIRONMENTAL MONITORING SYSTEM ===");
  Serial.println("Arduino GIGA");
  Serial.println("Airflow + 3x Soil Moisture + 2x TMP117 + BME280 + O2 + CO2");
  Serial.println("======================================\n");
  Serial.println("Soil1_%  Soil2_%  Soil3_%  Airflow_m_s  TMP1_C  TMP2_C  BME_T_C  BME_RH_%  BME_P_kPa  BME_Alt_m  O2_%  Light_IR  Light_Full  Light_Visible  Light_Lux");
  
  // Initialize soil moisture sensors
  pinMode(SOIL_1_PIN, INPUT);
  pinMode(SOIL_2_PIN, INPUT);
  pinMode(SOIL_3_PIN, INPUT);
  
  // Configure optional power control pins
  if (SOIL_1_PWR_PIN >= 0) {
    pinMode(SOIL_1_PWR_PIN, OUTPUT);
    digitalWrite(SOIL_1_PWR_PIN, HIGH);
  }
  if (SOIL_2_PWR_PIN >= 0) {
    pinMode(SOIL_2_PWR_PIN, OUTPUT);
    digitalWrite(SOIL_2_PWR_PIN, HIGH);
  }
  if (SOIL_3_PWR_PIN >= 0) {
    pinMode(SOIL_3_PWR_PIN, OUTPUT);
    digitalWrite(SOIL_3_PWR_PIN, HIGH);
  }

  // Initialize I2C communication
  Wire.begin();
  // Use fast-mode I2C for better throughput
  Wire.setClock(400000);

  // Initialize BME280 sensor (try address 0x77, then 0x76)
  bme.setI2CAddress(0x77);
  if (bme.beginI2C(Wire)) {
    bme_ok = true;
  } else {
    bme.setI2CAddress(0x76);
    bme_ok = bme.beginI2C(Wire);
  }

  if (bme_ok) {
    bme.setTempOverSample(4);
    bme.setPressureOverSample(4);
    bme.setHumidityOverSample(4);
    Serial.println("BME280 ready (I2C)");
  } else {
    Serial.println("BME280 not detected (0x77/0x76)");
  }

  // Initialize TMP117 sensors
  tmp117_1_ok = tmp117_1.begin(0x48, Wire);
  tmp117_2_ok = tmp117_2.begin(0x4A, Wire);

  if (tmp117_1_ok) {
    Serial.println("TMP117 #1 (0x48) OK");
  } else {
    Serial.println("TMP117 #1 (0x48) FAILED");
  }
  if (tmp117_2_ok) {
    Serial.println("TMP117 #2 (0x4A) OK");
  } else {
    Serial.println("TMP117 #2 (0x4A) FAILED");
  }

  // Initialize PASCO2 CO2 sensor
  if (co2.begin() == XENSIV_PASCO2_OK) {
    co2_ok = true;
    Serial.println("PASCO2 CO2 Sensor (0x28) OK");
    if (co2.startMeasure(10) != XENSIV_PASCO2_OK) {
      Serial.println("PASCO2 startMeasure failed.");
    }
  } else {
    Serial.println("PASCO2 init failed (check wiring/address).");
  }

  // Initialize TSL2591 light sensor
  if (tsl.begin()) {
    light_ok = true;
    Serial.println("TSL2591 Light Sensor (0x29) OK");
    // Configure gain and integration time
    tsl.setGain(TSL2591_GAIN_MED);
    tsl.setTiming(TSL2591_INTEGRATIONTIME_300MS);
  } else {
    Serial.println("TSL2591 init failed. Install library and check wiring.");
  }
}

// ============================================================================
// HELPER FUNCTIONS
// ============================================================================

/**
 * Prints all sensor readings in CSV format to Serial.
 * Output: soil1%, soil2%, soil3%, velocity_m/s
 */
void printCsvLine() {
  Serial.print(latest_soil1_percent, 2);
  Serial.print(", ");
  Serial.print(latest_soil2_percent, 2);
  Serial.print(", ");
  Serial.print(latest_soil3_percent, 2);
  Serial.print(", ");
  Serial.print(latest_velocity_m_s, 3);
  Serial.println();
}

// ============================================================================
// MAIN LOOP
// ============================================================================
/*
 * Continuously reads and processes sensor data.
 * Updates occur based on configured intervals.
 */
void loop() {
  // Process airflow sensor (1-second update interval)
  if (millis() - lastTime >= 1000) {
    // Read pulse count atomically
    noInterrupts();
    unsigned long pulses = pulseCount;
    pulseCount = 0;
    interrupts();
    
    pulseAccumulator += pulses;
    elapsedTime += 1000;
    
    // Calculate velocity when smoothing window is complete
    if (elapsedTime >= windowSec * 1000UL) {
      float liters = pulseAccumulator / K_FACTOR;
      float flow_L_min = liters * (60.0 / windowSec);
      float flow_m3_s = (flow_L_min / 1000.0) / 60.0;
      latest_velocity_m_s = flow_m3_s / AREA;
      
      // Reset for next measurement window
      pulseAccumulator = 0;
      elapsedTime = 0;
    }
    lastTime = millis();

    // Update latest BME280 data (no printing here)
    if (bme_ok) {
      latest_bme_temp_c = bme.readTempC();
      latest_bme_humidity = bme.readFloatHumidity();
      latest_bme_pressure_kpa = bme.readFloatPressure() / 1000.0f;
      latest_bme_altitude_m = bme.readFloatAltitudeMeters();
    }
  }
  
  // Update TMP117 readings when data is ready
  if (tmp117_1_ok && tmp117_1.dataReady()) {
    latest_tmp117_1_c = tmp117_1.readTempC();
  }
  if (tmp117_2_ok && tmp117_2.dataReady()) {
    latest_tmp117_2_c = tmp117_2.readTempC();
  }
  // Update Light readings
  if (light_ok) {
    uint32_t lum = tsl.getFullLuminosity();
    latest_light_ir = lum >> 16;
    latest_light_full = lum & 0xFFFF;
    latest_light_visible = latest_light_full - latest_light_ir;
    latest_light_lux = tsl.calculateLux(latest_light_full, latest_light_ir);
  }
  // Update CO2 ppm if available
  if (co2_ok) {
    int16_t co2ppm = 0;
    if (co2.getCO2(co2ppm) == XENSIV_PASCO2_OK && co2ppm > 0) {
      latest_co2_ppm = co2ppm;
    }
  }
  // Process soil moisture sensors
  unsigned long now = millis();
  
  if (now - soilLastMeasurement >= SOIL_MEAS_INTERVAL_MS) {
    soilLastMeasurement = now;

    // Read raw analog values from all three sensors
    int soil1_raw = analogRead(SOIL_1_PIN);
    int soil2_raw = analogRead(SOIL_2_PIN);
    int soil3_raw = analogRead(SOIL_3_PIN);

    // Convert to percentages
    latest_soil1_percent = rawToPercent(soil1_raw);
    latest_soil2_percent = rawToPercent(soil2_raw);
    latest_soil3_percent = rawToPercent(soil3_raw);

    // No printing here; unified print handled by scheduler below
  }

  // O2 sensor update with warmup and interval
  unsigned long now_ms = millis();
  if (!warmedUp && (now_ms - startupTime >= WARMUP_TIME_MS)) {
    warmedUp = true;
  }
  if (warmedUp && (now_ms - o2LastMeasurement >= O2_MEAS_INTERVAL_MS)) {
    o2LastMeasurement = now_ms;
    int o2_raw = analogRead(O2_PIN);
    float o2_voltage = (o2_raw / (float)ADC_MAX_VALUE) * ADC_REF_VOLTAGE;
    // Convert voltage to O2 % (inverse linear relationship)
    float span = (O2_VOLTAGE_AT_0_PERCENT - O2_VOLTAGE_AT_25_PERCENT);
    float pct = ((O2_VOLTAGE_AT_0_PERCENT - o2_voltage) / span) * O2_PERCENT_MAX;
    if (pct < 0.0) pct = 0.0;
    if (pct > O2_PERCENT_MAX) pct = O2_PERCENT_MAX;
    latest_o2_percent = pct;
  }

  // Scheduled prints: every 10 seconds, each reading on its own line
  if (now_ms - lastCombinedPrint >= PRINT_INTERVAL_MS) {
    lastCombinedPrint = now_ms;
    Serial.print("Soil1_%: "); Serial.println(latest_soil1_percent, 2);
    Serial.print("Soil2_%: "); Serial.println(latest_soil2_percent, 2);
    Serial.print("Soil3_%: "); Serial.println(latest_soil3_percent, 2);
    Serial.print("Airflow_m_s: "); Serial.println(latest_velocity_m_s, 3);
    Serial.print("TMP1_C: "); Serial.println(latest_tmp117_1_c, 2);
    Serial.print("TMP2_C: "); Serial.println(latest_tmp117_2_c, 2);
    Serial.print("BME_T_C: "); Serial.println(latest_bme_temp_c, 2);
    Serial.print("BME_RH_%: "); Serial.println(latest_bme_humidity, 1);
    Serial.print("BME_P_kPa: "); Serial.println(latest_bme_pressure_kpa, 3);
    Serial.print("BME_Alt_m: "); Serial.println(latest_bme_altitude_m, 1);
    Serial.print("O2_%: "); Serial.println(latest_o2_percent, 2);
    Serial.print("CO2_ppm: "); Serial.println(latest_co2_ppm);
    Serial.print("Light_IR: "); Serial.println(latest_light_ir);
    Serial.print("Light_Full: "); Serial.println(latest_light_full);
    Serial.print("Light_Visible: "); Serial.println(latest_light_visible);
    Serial.print("Light_Lux: "); Serial.println(latest_light_lux, 2);
  }

  delay(10);
}

// ============================================================================
// INTERRUPT SERVICE ROUTINES
// ============================================================================

/**
 * Airflow sensor pulse counter (ISR).
 * Increments pulse count on each rising edge.
 */
void countPulse() {
  pulseCount++;
}

// ============================================================================
// UTILITY FUNCTIONS
// ============================================================================

/**
 * Converts raw ADC reading to percentage.
 * 
 * @param rawValue: Raw ADC value (0-1023)
 * @return: Percentage (0.0-100.0)
 */
float rawToPercent(int rawValue) {
  return (rawValue / 1023.0) * 100.0;
}
