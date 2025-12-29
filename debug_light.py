#!/usr/bin/python3
import time
import sys
import os

# Import your existing module
try:
    from modules.light_sensor import LightSensor
except ImportError:
    print("ERROR: Could not find 'modules/light_sensor.py'.")
    print("Make sure you are running this from the 'FYP_Robot' folder.")
    sys.exit()

def debug_loop():
    print("--- LIGHT SENSOR DEBUG TOOL ---")
    print("1. Ensure Sensor is plugged into GPIO 23.")
    print("2. Cover the sensor with your hand to test.")
    print("3. Press Ctrl+C to exit.\n")

    # Initialize
    sensor = LightSensor(pin=23)

    try:
        while True:
            # Get raw status
            is_dark = sensor.is_dark()
            
            # Print status with visual indicator
            if is_dark:
                print(f"Status: [ DARK  ] (Value: 1) -> Robot would PAUSE")
            else:
                print(f"Status: [ LIGHT ] (Value: 0) -> Robot would WORK")
            
            time.sleep(0.5)

    except KeyboardInterrupt:
        print("\nExiting Debug Tool.")
        sensor.cleanup()

if __name__ == "__main__":
    debug_loop()