#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Ultrasonic Sensor Module (HC-SR04)
Measures distance using sound waves: sends pulse → waits for echo → calculates distance
Formula: Distance = (Time × Speed of Sound) / 2
"""

import lgpio  # GPIO control library for Raspberry Pi
import time

# --- GPIO Pin Configuration ---
TRIG_PIN = 28  # Output: sends ultrasonic trigger pulse
ECHO_PIN = 29  # Input: receives echo signal
OBSTACLE_DISTANCE_THRESHOLD = 5  # cm - objects closer than this = obstacle
DISTANCE_BUFFER_SIZE = 5  # Number of readings to average (noise reduction)


class UltrasonicSensor:
    """Controls ultrasonic sensor for distance measurement and obstacle detection"""
    
    def __init__(self, trig_pin=TRIG_PIN, echo_pin=ECHO_PIN, obstacle_threshold=OBSTACLE_DISTANCE_THRESHOLD):
        """Initialize the ultrasonic sensor
        
        Args:
            trig_pin (int): GPIO pin for trigger
            echo_pin (int): GPIO pin for echo
            obstacle_threshold (int): Distance threshold in cm for obstacle detection
        """
        self.trig_pin = trig_pin
        self.echo_pin = echo_pin
        self.obstacle_threshold = obstacle_threshold
        self.last_distance = None
        self.distance_history = []  # Buffer for averaging readings
        self.consecutive_detections = 0  # Counter for noise filtering
        
        try:
            # Open GPIO chip and configure pins
            self.h = lgpio.gpiochip_open(0)
            lgpio.gpio_claim_output(self.h, self.trig_pin)  # TRIG = output
            lgpio.gpio_claim_input(self.h, self.echo_pin)   # ECHO = input
            lgpio.gpio_write(self.h, self.trig_pin, 0)      # Start with TRIG LOW
            time.sleep(0.5)  # Let sensor stabilize
            print(f"✅ Ultrasonic Sensor initialized (Obstacle threshold: {obstacle_threshold}cm)")
        except Exception as e:
            print(f"⚠️ WARNING: Could not initialize ultrasonic sensor: {e}")
            self.h = None
    
    def get_distance(self):
        """
        Measure distance to nearest object.
        Steps: 
        1) Send 10μs trigger pulse  
        2) Wait for echo HIGH  
        3) Wait for echo LOW  4) Calculate distance
        Returns: distance in cm, or None if failed
        """
        if self.h is None:
            return None
        
        try:
            # Step 1: Send 10μs trigger pulse
            lgpio.gpio_write(self.h, self.trig_pin, 1)
            time.sleep(0.00001)  # 10 microseconds
            lgpio.gpio_write(self.h, self.trig_pin, 0)
            
            time.sleep(0.00005)  # Wait for sensor to stabilize
            
            # Step 2: Wait for ECHO to go HIGH (max 50ms timeout)
            timeout = 0.05
            timeout_start = time.time()
            while lgpio.gpio_read(self.h, self.echo_pin) == 0:
                if time.time() - timeout_start > timeout:
                    return None  # Timeout, no echo received
            start_time = time.time()  # Capture when echo goes HIGH
            
            # Step 3: Wait for ECHO to go LOW
            while lgpio.gpio_read(self.h, self.echo_pin) == 1:
                if time.time() - start_time > timeout:
                    return None  # Timeout, echo didn't end properly
            stop_time = time.time()  # Capture when echo goes LOW
            
            # Step 4: Calculate distance (speed of sound = 34300 cm/s, divide by 2 for round trip)
            time_elapsed = stop_time - start_time
            distance = (time_elapsed * 34300) / 2
            self.last_distance = round(distance, 2)
            return self.last_distance
        except Exception as e:
            print(f"⚠️ WARNING: Error reading ultrasonic sensor: {e}")
            return None
    
    def is_obstacle_detected(self):
        """
        Check if obstacle is within threshold, with noise filtering.
        Uses averaging + consecutive detection to prevent false positives.
        Returns: True if obstacle consistently detected, False otherwise
        """
        distance = self.get_distance()
        if distance is None:
            self.consecutive_detections = 0
            return False
        
        # Add to buffer, maintain max size
        self.distance_history.append(distance)
        if len(self.distance_history) > DISTANCE_BUFFER_SIZE:
            self.distance_history.pop(0)
        
        # Average readings for stability
        avg_distance = sum(self.distance_history) / len(self.distance_history)
        
        # Require 2+ consecutive detections to confirm obstacle
        if avg_distance < self.obstacle_threshold:
            self.consecutive_detections += 1
            return self.consecutive_detections >= 2
        else:
            self.consecutive_detections = 0
            return False
    
    def cleanup(self):
        """Release GPIO resources - call this when done"""
        try:
            if self.h is not None:
                lgpio.gpiochip_close(self.h)
                print("✅ Ultrasonic sensor cleaned up")
        except Exception as e:
            print(f"⚠️ WARNING: Error cleaning up ultrasonic sensor: {e}")


# --- Test Loop (run directly to test sensor) ---
if __name__ == "__main__":
    sensor = UltrasonicSensor()
    try:
        while True:
            obstacle = sensor.is_obstacle_detected()
            
            # Use the last measured distance (already obtained in is_obstacle_detected)
            if sensor.last_distance is not None:
                status = "🚨 OBSTACLE DETECTED!" if obstacle else "✅ Clear"
                print(f"Distance: {sensor.last_distance}cm - {status}")
            else:
                print("Sensor unavailable")
            time.sleep(0.2)
    except KeyboardInterrupt:
        print("\nProgram stopped by user")
    finally:
        sensor.cleanup()
