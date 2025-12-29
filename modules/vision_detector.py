#!/usr/bin/python3
import cv2
import math
import time
import os
import sys
import numpy as np
from ultralytics import YOLO

# --- PATH SETUP ---
# Ensure we can find the robot drivers
FACTORY_PATH = '/home/pi/TonyPi/HiwonderSDK'
ROOT_PATH = '/home/pi/TonyPi'
sys.path.append(FACTORY_PATH)
sys.path.append(ROOT_PATH)

# --- DRIVER IMPORTS ---
try:
    import hiwonder.ros_robot_controller_sdk as rrc
    from hiwonder.Controller import Controller
    import hiwonder.yaml_handle as yaml_handle
    import hiwonder.Camera as Camera # Use Factory Camera Driver if needed
    board = rrc.Board()
    ctl = Controller(board)
except ImportError:
    # Fallback for debugging on PC
    import hiwonder.Board as Board
    ctl = Board
    yaml_handle = None
    print("[WARN] Robot Drivers not found. Running in simulation mode.")

# --- PID CONTROLLER CLASS ---
class PID:
    def __init__(self, P=0.2, I=0.0, D=0.0):
        self.Kp, self.Ki, self.Kd = P, I, D
        self.sample_time = 0.00
        self.current_time = time.time()
        self.last_time = self.current_time
        self.clear()
        
    def clear(self):
        self.SetPoint = 0.0
        self.PTerm = 0.0
        self.ITerm = 0.0
        self.DTerm = 0.0
        self.last_error = 0.0
        self.int_error = 0.0
        self.windup_guard = 20.0
        self.output = 0.0
        
    def update(self, feedback_value):
        error = self.SetPoint - feedback_value
        self.current_time = time.time()
        delta_time = self.current_time - self.last_time
        delta_error = error - self.last_error
        if (delta_time >= self.sample_time):
            self.PTerm = self.Kp * error
            self.ITerm += error * delta_time
            if (self.ITerm < -self.windup_guard): self.ITerm = -self.windup_guard
            elif (self.ITerm > self.windup_guard): self.ITerm = self.windup_guard
            self.DTerm = 0.0
            if delta_time > 0: self.DTerm = delta_error / delta_time
            self.last_time = self.current_time
            self.last_error = error
            self.output = self.PTerm + (self.Ki * self.ITerm) + (self.Kd * self.DTerm)

# --- CONFIGURATION ---
IMG_W, IMG_H = 640, 480
CENTER_X, CENTER_Y = IMG_W // 2, IMG_H // 2
# PID Gains for Head Tracking
pid_X = PID(P=0.06, I=0.005, D=0.005) 
pid_Y = PID(P=0.06, I=0.005, D=0.005) 
# Servo IDs and Ranges
PAN_SERVO_ID, TILT_SERVO_ID = 1, 2
PAN_RANGE, TILT_RANGE = (500, 2500), (1200, 2000)

class VisionDetector:
    def __init__(self):
        # 1. LOAD YOLO MODEL
        current_dir = os.path.dirname(os.path.abspath(__file__))
        model_path = os.path.join(current_dir, "cardboard_v1.pt") 
        try:
            self.model = YOLO(model_path)
            print("[VISION] YOLO Model Loaded Successfully")
        except:
            print(f"[VISION] ERROR: Model not found at {model_path}")

        # 2. OPEN CAMERA (ROBUST LOGIC)
        self.camera = None
        CAMERA_INDEX = 0  # <--- Change this to 1 if you found it on index 1
        
        print(f"[VISION] Attempting Camera Index {CAMERA_INDEX}...")
        self.camera = cv2.VideoCapture(CAMERA_INDEX)

        # Fallback Check: If hardware failed, switch to Stream
        if not self.camera.isOpened():
            print("[VISION] Hardware busy. Switching to stream.")
            self.camera = cv2.VideoCapture('http://127.0.0.1:8080/?action=stream?dummy=param.mjpg')
        else:
            # Only set resolution if using hardware driver
            try:
                self.camera.set(3, IMG_W)
                self.camera.set(4, IMG_H)
            except: pass
            print("[VISION] Camera Connected.")

        # 3. INITIALIZE HEAD POSITION
        self.pan_pulse, self.tilt_pulse = 1500, 1500
        self.move_head(self.pan_pulse, self.tilt_pulse)

    def move_head(self, pan, tilt):
        """Safely moves the head servos"""
        pan = max(PAN_RANGE[0], min(pan, PAN_RANGE[1]))
        tilt = max(TILT_RANGE[0], min(tilt, TILT_RANGE[1]))
        
        # Support both Controller types (Pro vs Standard)
        if hasattr(ctl, 'set_pwm_servo_pulse'):
            ctl.set_pwm_servo_pulse(PAN_SERVO_ID, int(pan), 20)
            ctl.set_pwm_servo_pulse(TILT_SERVO_ID, int(tilt), 20)
        else:
            ctl.setPWMServoPulse(PAN_SERVO_ID, int(pan), 20)
            ctl.setPWMServoPulse(TILT_SERVO_ID, int(tilt), 20)
            
        self.pan_pulse, self.tilt_pulse = pan, tilt

    def get_frame(self):
        """Helper to let main.py borrow the camera frame"""
        if self.camera and self.camera.isOpened():
            ret, frame = self.camera.read()
            return ret, frame
        return False, None

    def track_object(self):
        """
        Runs inference, updates head position, and returns Lock Status.
        Returns: (is_locked, coords)
        """
        ret, frame = self.get_frame()
        if not ret or frame is None:
            return False, {'x': 0.5, 'y': 0.5}

        # Run Inference
        results = self.model(frame, stream=True, verbose=False)
        box_center = None
        
        # Parse Results
        for r in results:
            for box in r.boxes:
                # Confidence Threshold
                if box.conf[0] > 0.5:
                    x1, y1, x2, y2 = box.xyxy[0]
                    cx, cy = int((x1+x2)/2), int((y1+y2)/2)
                    box_center = (cx, cy)
                    
                    # Update Head PID
                    pid_X.SetPoint, pid_Y.SetPoint = CENTER_X, CENTER_Y
                    pid_X.update(cx)
                    pid_Y.update(cy)
                    
                    # Move Head
                    self.pan_pulse -= pid_X.output 
                    self.tilt_pulse += pid_Y.output
                    self.move_head(self.pan_pulse, self.tilt_pulse)
                    
                    # Only track the first object found
                    break 
            if box_center: break
        
        # Check Lock Status
        if box_center:
            # "Locked" means the object is close to the center of the screen
            locked = abs(box_center[0]-CENTER_X) < 30 and abs(box_center[1]-CENTER_Y) < 30
            # Return normalized coordinates (0.0 to 1.0)
            return locked, {'x': box_center[0]/IMG_W, 'y': box_center[1]/IMG_H}
        
        # Object not found
        return False, {'x': 0.5, 'y': 0.5}