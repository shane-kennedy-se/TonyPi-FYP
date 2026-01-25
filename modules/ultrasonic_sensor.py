#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
================================================================================
ULTRASONIC SENSOR MODULE - HC-SR04 Distance Measurement
================================================================================
This module provides distance measurement and obstacle detection functionality
using an HC-SR04 ultrasonic sensor connected to the TonyPi robot.

HOW ULTRASONIC SENSORS WORK:
----------------------------
1. The sensor emits an ultrasonic sound pulse (40kHz, inaudible to humans)
2. The pulse travels through air, hits an object, and bounces back
3. The sensor detects the returning echo
4. By measuring the time between sending and receiving, we calculate distance

PHYSICS FORMULA:
----------------
Distance = (Time × Speed of Sound) / 2
- Speed of sound in air ≈ 343 m/s = 34,300 cm/s
- We divide by 2 because the sound travels TO the object AND back

GPIO PIN CONNECTION:
--------------------
- TRIG (Trigger) Pin: Sends the ultrasonic pulse when set HIGH for 10μs
- ECHO Pin: Goes HIGH when pulse is sent, goes LOW when echo is received

USAGE IN ROBOT:
---------------
The ultrasonic sensor is mounted on the robot to detect obstacles in front.
When an obstacle is detected within the threshold distance, the robot can
stop or take evasive action to avoid collision.

Author: FYP Project
================================================================================
"""

import lgpio      # Linux GPIO library - provides low-level GPIO control on Raspberry Pi
import time       # Used for precise timing measurements and delays

# ================================================================================
# GPIO PIN CONFIGURATION
# ================================================================================
# These pin numbers correspond to the physical GPIO pins on the Raspberry Pi
# where the HC-SR04 ultrasonic sensor is connected.
#
# PIN MAPPING (BCM numbering):
# - GPIO 28 → TRIG pin of HC-SR04 (OUTPUT - we send trigger pulse)
# - GPIO 29 → ECHO pin of HC-SR04 (INPUT - we receive echo response)
# ================================================================================
TRIG_PIN = 28
ECHO_PIN = 29

# ================================================================================
# SENSOR CONFIGURATION CONSTANTS
# ================================================================================
# OBSTACLE_DISTANCE_THRESHOLD: If an object is closer than this distance (cm),
# it is considered an obstacle. This value can be tuned based on robot's needs.
# - Lower value (e.g., 5cm): Robot gets very close before detecting obstacle
# - Higher value (e.g., 30cm): Robot detects obstacle from further away
#
# DISTANCE_BUFFER_SIZE: Number of consecutive readings to average together.
# This helps reduce noise and false readings caused by sensor inaccuracy.
# - Higher value: More stable readings, but slower response time
# - Lower value: Faster response, but more susceptible to noise
# ================================================================================
OBSTACLE_DISTANCE_THRESHOLD = 5   # cm - obstacle detected if closer than this
DISTANCE_BUFFER_SIZE = 5          # Number of readings to average for noise reduction


class UltrasonicSensor:
    """
    ================================================================================
    UltrasonicSensor Class
    ================================================================================
    This class encapsulates all functionality for the HC-SR04 ultrasonic sensor.
    It provides methods to:
    1. Initialize the GPIO pins
    2. Measure distance to objects
    3. Detect obstacles with noise filtering
    4. Clean up resources when done
    
    KEY FEATURES:
    - Timeout handling: Prevents infinite loops if sensor fails
    - Noise filtering: Uses averaging and consecutive detection to reduce false positives
    - Error handling: Gracefully handles sensor failures without crashing
    ================================================================================
    """
    
    def __init__(self, trig_pin=TRIG_PIN, echo_pin=ECHO_PIN, obstacle_threshold=OBSTACLE_DISTANCE_THRESHOLD):
        """
        Initialize the ultrasonic sensor and configure GPIO pins.
        
        This method:
        1. Stores the configuration parameters
        2. Opens a connection to the GPIO chip
        3. Configures TRIG pin as OUTPUT (we send signals)
        4. Configures ECHO pin as INPUT (we receive signals)
        5. Sets TRIG to LOW and waits for sensor to stabilize
        
        Args:
            trig_pin (int): GPIO pin number for trigger (default: 28)
            echo_pin (int): GPIO pin number for echo (default: 29)
            obstacle_threshold (int): Distance in cm below which obstacle is detected
        
        GPIO Pin Modes:
            - OUTPUT: We control this pin, sending HIGH/LOW signals
            - INPUT: We read this pin to detect HIGH/LOW states from sensor
        """
        # Store configuration in instance variables for later use
        self.trig_pin = trig_pin
        self.echo_pin = echo_pin
        self.obstacle_threshold = obstacle_threshold
        
        # last_distance: Stores the most recent successful distance reading
        # Used for debugging and to check sensor status
        self.last_distance = None
        
        # distance_history: A list (buffer) storing recent distance readings
        # Used to calculate an average and filter out noise/outliers
        self.distance_history = []
        
        # consecutive_detections: Counter for how many times in a row
        # an obstacle has been detected. Used to prevent false positives.
        # We only report an obstacle if detected multiple times consecutively.
        self.consecutive_detections = 0
        
        try:
            # ============================================================
            # GPIO INITIALIZATION
            # ============================================================
            # Open the GPIO chip - this gives us a "handle" (h) to control pins
            # gpiochip_open(0) opens the default GPIO chip on Raspberry Pi
            self.h = lgpio.gpiochip_open(0)
            
            # Configure TRIG pin as OUTPUT
            # We will write HIGH/LOW values to this pin to trigger measurements
            lgpio.gpio_claim_output(self.h, self.trig_pin)
            
            # Configure ECHO pin as INPUT
            # We will read this pin to measure how long the echo takes
            lgpio.gpio_claim_input(self.h, self.echo_pin)
            
            # Set TRIG to LOW initially (no trigger pulse yet)
            lgpio.gpio_write(self.h, self.trig_pin, 0)
            
            # Wait 500ms for sensor to stabilize after power-on
            # HC-SR04 requires a brief settling time before first measurement
            time.sleep(0.5)
            
            print(f"✅ Ultrasonic Sensor initialized (Obstacle threshold: {obstacle_threshold}cm)")
            
        except Exception as e:
            # If GPIO initialization fails (e.g., wrong pins, permissions issue),
            # we set self.h to None so other methods know the sensor is unavailable
            print(f"⚠️ WARNING: Could not initialize ultrasonic sensor: {e}")
            self.h = None
    
    def get_distance(self):
        """
        Measure the distance to the nearest object in front of the sensor.
        
        This method implements the HC-SR04 measurement protocol:
        
        STEP 1: SEND TRIGGER PULSE
        --------------------------
        - Set TRIG pin HIGH for exactly 10 microseconds
        - This tells the sensor to emit an ultrasonic burst (8 pulses at 40kHz)
        
        STEP 2: WAIT FOR ECHO START
        ---------------------------
        - After trigger, the ECHO pin will go HIGH when the sensor starts listening
        - We wait for ECHO to go HIGH (with timeout to prevent infinite loop)
        - Record the start time when ECHO goes HIGH
        
        STEP 3: WAIT FOR ECHO END
        -------------------------
        - ECHO pin goes LOW when the ultrasonic pulse returns
        - We wait for ECHO to go LOW (with timeout)
        - Record the stop time when ECHO goes LOW
        
        STEP 4: CALCULATE DISTANCE
        --------------------------
        - Time elapsed = stop_time - start_time
        - Distance = (time × 34300 cm/s) / 2
        - Divide by 2 because sound travels there AND back
        
        Returns:
            float: Distance in centimeters (rounded to 2 decimal places)
            None: If sensor is unavailable or measurement fails
        
        Timing Diagram:
                          ___________
        TRIG: ___________| 10μs     |___________
                                ________________
        ECHO: _________________|  time elapsed  |___
                               ↑                ↑
                            start             stop
        """
        # Check if sensor was initialized successfully
        if self.h is None:
            return None
        
        try:
            # ============================================================
            # STEP 1: SEND 10 MICROSECOND TRIGGER PULSE
            # ============================================================
            # Set TRIG pin HIGH to start the trigger pulse
            lgpio.gpio_write(self.h, self.trig_pin, 1)
            
            # Wait exactly 10 microseconds (0.00001 seconds)
            # This is the required pulse width for HC-SR04
            time.sleep(0.00001)
            
            # Set TRIG pin LOW to end the trigger pulse
            lgpio.gpio_write(self.h, self.trig_pin, 0)
            
            # ============================================================
            # STABILIZATION DELAY (CRITICAL FIX)
            # ============================================================
            # Wait 50 microseconds for sensor to stabilize after trigger
            # The sensor needs time to:
            # - Transmit the ultrasonic burst (8 pulses)
            # - Switch to receiving mode
            # Without this delay, we might catch the outgoing pulse as echo
            time.sleep(0.00005)
            
            # ============================================================
            # STEP 2: WAIT FOR ECHO TO START (GO HIGH)
            # ============================================================
            # Timeout of 50ms corresponds to max detectable distance of ~8.5 meters
            # Formula: 0.05s × 34300 cm/s / 2 = 857.5 cm ≈ 8.5m
            timeout = 0.05
            timeout_start = time.time()
            
            # Wait in a loop until ECHO pin goes HIGH
            # gpio_read returns 0 (LOW) or 1 (HIGH)
            while lgpio.gpio_read(self.h, self.echo_pin) == 0:
                # Check if we've waited too long - no echo detected
                if time.time() - timeout_start > timeout:
                    return None  # Timeout: either no object or sensor error
            
            # ECHO just went HIGH - record the start time
            start_time = time.time()
            
            # ============================================================
            # STEP 3: WAIT FOR ECHO TO END (GO LOW)
            # ============================================================
            # Keep waiting while ECHO is still HIGH
            while lgpio.gpio_read(self.h, self.echo_pin) == 1:
                # Check for timeout - echo taking too long means distant object
                if time.time() - start_time > timeout:
                    return None  # Timeout: object too far or sensor error
            
            # ECHO just went LOW - record the stop time
            stop_time = time.time()
            
            # ============================================================
            # STEP 4: CALCULATE DISTANCE
            # ============================================================
            # Calculate the time the echo took to return
            time_elapsed = stop_time - start_time
            
            # Calculate distance using speed of sound (343 m/s = 34300 cm/s)
            # Divide by 2 because sound travels to object AND back
            # Distance = (Time × Speed) / 2
            distance = (time_elapsed * 34300) / 2
            
            # Round to 2 decimal places for cleaner output
            self.last_distance = round(distance, 2)
            
            return self.last_distance
            
        except Exception as e:
            print(f"⚠️ WARNING: Error reading ultrasonic sensor: {e}")
            return None
    
    def is_obstacle_detected(self):
        """
        Check if an obstacle is detected within the threshold distance.
        
        This method implements NOISE FILTERING to prevent false positives:
        
        1. AVERAGING: Maintains a buffer of recent readings and uses
           the average to smooth out individual noisy readings.
        
        2. CONSECUTIVE DETECTION: Requires the obstacle to be detected
           at least 2 times in a row before reporting it as real.
           This prevents single erroneous readings from triggering
           false obstacle alerts.
        
        WHY NOISE FILTERING IS IMPORTANT:
        ---------------------------------
        Ultrasonic sensors can produce false readings due to:
        - Echoes from angled surfaces bouncing away
        - Soft materials (clothing, foam) absorbing sound
        - Environmental noise from other ultrasonic sources
        - Interference patterns from multiple reflections
        
        Without filtering, the robot might:
        - Stop unexpectedly when there's no real obstacle
        - Fail to detect a real obstacle due to one bad reading
        
        Returns:
            bool: True if obstacle is consistently detected within threshold
                  False if no obstacle or readings are inconsistent
        
        Algorithm Flow:
        ---------------
        1. Get new distance reading
        2. Add to history buffer (keep only last N readings)
        3. Calculate average of all readings in buffer
        4. If average < threshold, increment consecutive counter
        5. Return True only if consecutive counter >= 2
        """
        # Get a new distance measurement
        distance = self.get_distance()
        
        # If measurement failed, reset consecutive counter and return False
        if distance is None:
            self.consecutive_detections = 0
            return False
        
        # ============================================================
        # STEP 1: ADD TO HISTORY BUFFER FOR AVERAGING
        # ============================================================
        # Add the new reading to the history list
        self.distance_history.append(distance)
        
        # If buffer exceeds max size, remove the oldest reading (FIFO queue)
        if len(self.distance_history) > DISTANCE_BUFFER_SIZE:
            self.distance_history.pop(0)  # Remove first (oldest) element
        
        # ============================================================
        # STEP 2: CALCULATE AVERAGE FOR NOISE REDUCTION
        # ============================================================
        # Sum all readings and divide by count to get average
        # This smooths out individual noisy readings
        avg_distance = sum(self.distance_history) / len(self.distance_history)
        
        # ============================================================
        # STEP 3: CHECK THRESHOLD WITH CONSECUTIVE DETECTION
        # ============================================================
        if avg_distance < self.obstacle_threshold:
            # Obstacle detected - increment consecutive counter
            self.consecutive_detections += 1
            
            # Only return True if detected at least 2 times in a row
            # This prevents single bad readings from triggering false alarms
            return self.consecutive_detections >= 2
        else:
            # No obstacle - reset the consecutive counter
            self.consecutive_detections = 0
            return False
    
    def cleanup(self):
        """
        Clean up GPIO resources when done using the sensor.
        
        This method should ALWAYS be called when the program exits to:
        1. Release the GPIO pins for use by other programs
        2. Reset pins to a safe state
        3. Free system resources
        
        IMPORTANT: Failing to call cleanup can cause:
        - GPIO pins to remain locked
        - Other programs unable to access the pins
        - Resource leaks over time
        
        Best Practice: Use try/finally or context managers to ensure
        cleanup is called even if the program crashes.
        """
        try:
            if self.h is not None:
                # Close the GPIO chip handle, releasing all claimed pins
                lgpio.gpiochip_close(self.h)
                print("✅ Ultrasonic sensor cleaned up")
        except Exception as e:
            print(f"⚠️ WARNING: Error cleaning up ultrasonic sensor: {e}")


# ================================================================================
# MAIN TEST LOOP (for standalone testing)
# ================================================================================
# This section only runs when the file is executed directly (not imported).
# It provides a simple test loop to verify the sensor is working correctly.
#
# USAGE:
#   python3 ultrasonic_sensor.py
#
# OUTPUT:
#   Displays continuous distance readings and obstacle status
#   Press Ctrl+C to stop the test
# ================================================================================
if __name__ == "__main__":
    # Create sensor instance with default settings
    sensor = UltrasonicSensor()
    
    try:
        # Infinite loop for continuous monitoring
        while True:
            # ============================================================
            # EFFICIENT MEASUREMENT APPROACH
            # ============================================================
            # is_obstacle_detected() internally calls get_distance(), so we
            # only call it once to avoid redundant sensor measurements.
            # Making multiple measurements per loop would:
            # - Waste time (each measurement takes ~10-60ms)
            # - Potentially give inconsistent readings
            obstacle = sensor.is_obstacle_detected()
            
            # Use the last measured distance (stored in sensor.last_distance)
            # This was already obtained during is_obstacle_detected()
            if sensor.last_distance is not None:
                # Format status message with emoji for visual clarity
                status = "🚨 OBSTACLE DETECTED!" if obstacle else "✅ Clear"
                print(f"Distance: {sensor.last_distance}cm - {status}")
            else:
                print("Sensor unavailable")
            
            # Wait 200ms between measurements
            # HC-SR04 recommends at least 60ms between readings
            # We use 200ms for comfortable visual output rate
            time.sleep(0.2)
    
    except KeyboardInterrupt:
        # Ctrl+C was pressed - exit gracefully
        print("\nProgram stopped by user")
    
    finally:
        # ALWAYS clean up GPIO resources, even if an error occurred
        # finally block executes whether try succeeded or raised exception
        sensor.cleanup()
