# Sproutnik Environmental Monitoring System

Arduino-based sensor integration for atmospheric and environmental data collection using the GIGA R1 WiFi board.

## Hardware Components

- **Arduino GIGA R1 WiFi** - Main controller
- **SparkFun BME280** - Temperature, humidity, pressure, altitude (I2C: 0x76/0x77)
- **SparkFun TMP117** - High-precision temperature sensors x2 (I2C: 0x48, 0x4A)
- **XENSIV PAS CO2** - CO₂ concentration sensor (I2C: 0x28)
- **Adafruit TSL2591** - Light/luminosity sensor (I2C: 0x29)
- **PFA Fixed Flow Meter (Seeed)** - Flow measurement (Digital: D2 pulse)
- **Grove O2 Sensor Pro (GGC2330-O2)** - Oxygen concentration (Analog: A5)
- **Sparkfun Soil Moisture Sensor** (Analog: A0, A1, A2)

## Wiring

All I2C sensors share the default bus:
- **SDA**: D20
- **SCL**: D21
- **Power**: 3.3V (BME280, TSL2591) or 5V (TMP117, PASCO2 as specified)
- **GND**: Common ground

PFA Fixed Flow Meter (pulse output):
- **VCC**: 5V
- **GND**: GND
- **Pulse OUT**: D2

Grove O2 Sensor Pro:
- **VCC**: 5V
- **GND**: GND
- **Analog OUT**: A5

## Installation

1. Install the Arduino IDE (1.8.9+)
2. Install the GIGA R1 WiFi board package via Boards Manager
3. Install required libraries via Library Manager:
   - SparkFun BME280
   - SparkFun TMP117
   - Adafruit TSL2591
   - Adafruit Unified Sensor

4. Clone this repository:
   ```bash
   git clone https://github.com/Medoriku/Sproutnik.git
   cd Sproutnik
   ```

5. Open `Complete.ino` in Arduino IDE
6. Select **Arduino GIGA R1 WiFi** as your board
7. Upload the sketch

## Usage

- Open Serial Monitor at **115200 baud**
- Sensor readings print continuously
- Allow a short settling time after power-up

## File Structure

```
Complete/
├── Complete.ino              # Main sketch
├── Oxygen.cpp/.h             # O2 sensor driver
├── SparkFunBME280.cpp/.h     # BME280 atmospheric sensor driver
├── pas-co2-*.cpp/.hpp        # PAS CO2 sensor drivers
├── xensiv_pasco2*            # XENSIV PAS CO2 core library
└── README.md
```

## Sensor Addresses

Run an I2C scanner to verify addresses:
- **BME280**: 0x77
- **TMP117 #1**: 0x48
- **TMP117 #2**: 0x4A
- **PAS CO2**: 0x28
- **TSL2591**: 0x29

## Author

**Diyora Daminova**  
Created: January 21, 2026

## License

Open source - feel free to use and modify.
