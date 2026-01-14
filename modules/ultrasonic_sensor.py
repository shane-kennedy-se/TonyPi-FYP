#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import lgpio
import time

# ----------------------------
# GPIO Pin Configuration
# ----------------------------
TRIG_PIN = 28
ECHO_PIN = 29


# ----------------------------
# GPIO Setup
# ----------------------------
h = lgpio.gpiochip_open(0)

lgpio.gpio_claim_output(h, TRIG_PIN)
lgpio.gpio_claim_input(h, ECHO_PIN)

lgpio.gpio_write(h, TRIG_PIN, 0)
time.sleep(0.5)

# ----------------------------
# Ultrasonic Distance Function
# ----------------------------
def get_distance():

    # Send trigger pulse
    lgpio.gpio_write(h, TRIG_PIN, 1)
    time.sleep(0.00001)
    lgpio.gpio_write(h, TRIG_PIN, 0)

    start_time = time.time()
    stop_time = time.time()

    # Wait for echo start
    while lgpio.gpio_read(h, ECHO_PIN) == 0:
        start_time = time.time()

    # Wait for echo end
    while lgpio.gpio_read(h, ECHO_PIN) == 1:
        stop_time = time.time()

    time_elapsed = stop_time - start_time

    distance = (time_elapsed * 34300) / 2
    return round(distance, 2)

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
        lgpio.gpiochip_close(h)
