import cv2
import time
import os
import sys
import numpy as np
from ultralytics import YOLO

# --- FACTORY COMPATIBILITY ---
sys.path.append('/home/pi/TonyPi/HiwonderSDK')
try:
    import hiwonder.ros_robot_controller_sdk as rrc
    from hiwonder.Controller import Controller
    board = rrc.Board()
    ctl = Controller(board)
except:
    print("[VISION] Running in Simulation Mode (No Robot Hardware)")
    ctl = None

# --- PID & CONFIG ---
IMG_W, IMG_H = 640, 480
CENTER_X, CENTER_Y = IMG_W // 2, IMG_H // 2

class PID:
    def __init__(self, P=0.1, I=0.0, D=0.0):
        self.Kp, self.Ki, self.Kd = P, I, D
        self.clear()
    def clear(self):
        self.SetPoint = 0.0
        self.last_error = 0.0
        self.output = 0.0
    def update(self, feedback_value):
        error = self.SetPoint - feedback_value
        self.output = (self.Kp * error) + (self.Kd * (error - self.last_error))
        self.last_error = error
        return self.output

pid_X = PID(P=0.06, D=0.005)
pid_Y = PID(P=0.06, D=0.005)

class VisionDetector:
    def __init__(self):
        # 1. Load Model
        current_dir = os.path.dirname(os.path.abspath(__file__))
        model_path = os.path.join(current_dir, "cardboard_v1.pt")
        try:
            self.model = YOLO(model_path)
            print("[VISION] YOLO Loaded.")
        except:
            print("[VISION] Error: YOLO Model not found.")

        # 2. Connect to Camera (The Fix)
        self.camera = None
        
        # Attempt 1: Hardware Index 0
        self.camera = cv2.VideoCapture(0)
        if not self.camera.isOpened():
            print("[VISION] Camera 0 busy. Switching to Network Stream...")
            # Attempt 2: Localhost Stream (Works if mjpg_streamer is running)
            self.camera = cv2.VideoCapture('http://127.0.0.1:8080/?action=stream?dummy=param.mjpg')
        
        if self.camera.isOpened():
            print("[VISION] Camera Connected.")
        else:
            print("[VISION] CRITICAL: Could not connect to any camera.")

        # 3. Init Head
        self.pan, self.tilt = 1500, 1500
        self.move_head(1500, 1500)

    def move_head(self, pan, tilt):
        if ctl:
            pan = max(500, min(2500, int(pan)))
            tilt = max(1200, min(2500, int(tilt)))
            ctl.set_pwm_servo_pulse(1, pan, 20)
            ctl.set_pwm_servo_pulse(2, tilt, 20)
        self.pan, self.tilt = pan, tilt

    def get_frame(self):
        if self.camera and self.camera.isOpened():
            ret, frame = self.camera.read()
            return ret, frame
        return False, None

    def track_object(self):
        ret, frame = self.get_frame()
        if not ret or frame is None:
            return False, {'x': 0.5, 'y': 0.5}

        results = self.model(frame, stream=True, verbose=False)
        target = None
        
        for r in results:
            for box in r.boxes:
                if box.conf[0] > 0.5:
                    x1, y1, x2, y2 = box.xyxy[0]
                    cx, cy = int((x1+x2)/2), int((y1+y2)/2)
                    target = (cx, cy)
                    break
            if target: break
        
        if target:
            cx, cy = target
            # PID Update (Minus X for servo direction)
            self.pan -= pid_X.update(cx)
            self.tilt += pid_Y.update(cy)
            self.move_head(self.pan, self.tilt)
            
            # Lock condition: Target is near center
            locked = abs(cx - CENTER_X) < 40 and abs(cy - CENTER_Y) < 40
            return locked, {'x': cx/IMG_W, 'y': cy/IMG_H}
            
        return False, {'x': 0.5, 'y': 0.5}