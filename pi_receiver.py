#!/usr/bin/env python3
"""
Raspberry Pi Serial Data Receiver
Receives sensor data from Arduino GIGA via USB serial connection
"""

import serial
import time
from datetime import datetime

# Configuration
SERIAL_PORT = '/dev/ttyACM0'  # Change to /dev/ttyUSB0 if needed
BAUD_RATE = 115200
TIMEOUT = 1

def connect_serial(port, baudrate, timeout):
    """Establish serial connection"""
    try:
        ser = serial.Serial(port, baudrate, timeout=timeout)
        print(f"Connected to {port} at {baudrate} baud")
        return ser
    except serial.SerialException as e:
        print(f"Error connecting to {port}: {e}")
        return None

def read_data(ser):
    """Read and print incoming serial data"""
    print("Waiting for data from Arduino...")
    print("-" * 50)
    
    try:
        while True:
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8').strip()
                # Remove quotation marks from the line
                line = line.replace('"', '')
                if line:
                    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                    print(f"[{timestamp}] {line}")
    except KeyboardInterrupt:
        print("\n\nData collection stopped by user")
    except Exception as e:
        print(f"Error reading data: {e}")

def main():
    # Connect to Arduino
    ser = connect_serial(SERIAL_PORT, BAUD_RATE, TIMEOUT)
    
    if ser is None:
        print("Failed to connect. Check your connection and port.")
        return
    
    # Give Arduino time to initialize
    time.sleep(2)
    
    # Read incoming data
    read_data(ser)
    
    # Cleanup
    if ser.is_open:
        ser.close()
        print("Serial connection closed")

if __name__ == "__main__":
    main()
