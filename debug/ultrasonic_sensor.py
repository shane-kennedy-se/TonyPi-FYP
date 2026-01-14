#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import RPi.GPIO as GPIO
import time

# ----------------------------
# GPIO Pin Configuration
# ----------------------------
TRIG_PIN = 23
ECHO_PIN = 24

# ----------------------------
# GPIO Setup
# ----------------------------
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

GPIO.setup(TRIG_PIN, GPIO.OUT)
GPIO.setup(ECHO_PIN, GPIO.IN)

GPIO.output(TRIG_PIN, GPIO.LOW)
time.sleep(0.5)

# ----------------------------
# Ultrasonic Distance Function
# ----------------------------
def get_distance():
    """
    Measure distance using HC-SR04 ultrasonic sensor.
    Return distance in cm.
    """

    # Send trigger pulse
    GPIO.output(TRIG_PIN, GPIO.HIGH)
    time.sleep(0.00001)
    GPIO.output(TRIG_PIN, GPIO.LOW)

    start_time = time.time()
    stop_time = time.time()

    # Wait for echo start
    while GPIO.input(ECHO_PIN) == 0:
        start_time = time.time()

    # Wait for echo end
    while GPIO.input(ECHO_PIN) == 1:
        stop_time = time.time()

    # Time difference
    time_elapsed = stop_time - start_time

    # Distance calculation (speed of sound = 34300 cm/s)
    distance = (time_elapsed * 34300) / 2
    distance = round(distance, 2)

    return distance


# ----------------------------
# Main Test Loop
# ----------------------------
if __name__ == "__main__":
    try:
        while True:
            dist = get_distance()
            print("Ultrasonic Distance:", dist, "cm")
            time.sleep(0.2)

    except KeyboardInterrupt:
        print("\nProgram stopped by user")

    finally:
        GPIO.cleanup()
