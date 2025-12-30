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
    print("[VISION] Simulation Mode (No Servos)")
    ctl = None

# --- CONSTANTS ---
IMG_W, IMG_H = 640, 480
CENTER_X, CENTER_Y = IMG_W // 2, IMG_H // 2

# --- ABSOLUTE MODEL PATH (CRITICAL FIX) ---
MODEL_PATH = "/home/pi/FYP_Robot/modules/cardboard_v1.pt"

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
        # 1. Load Model (With explicit check)
        if os.path.exists(MODEL_PATH):
            try:
                self.model = YOLO(MODEL_PATH)
                print(f"[VISION] YOLO Loaded from {MODEL_PATH}")
            except Exception as e:
                print(f"[VISION ERROR] Model corrupt? {e}")
                self.model = None
        else:
            print(f"[VISION ERROR] Model NOT FOUND at {MODEL_PATH}")
            self.model = None

        # 2. Connect to Camera (Direct Hardware Access)
        self.camera = cv2.VideoCapture(0)
        
        # Give it a second to wake up
        time.sleep(1)
        
        if self.camera.isOpened():
            print("[VISION] Camera Connected (Index 0).")
        else:
            print("[VISION CRITICAL] Camera 0 Failed. Is mjpg_streamer running?")

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
            if ret: return True, frame
        return False, None

    def track_object(self):
        # Safety check: if model didn't load, don't crash
        if self.model is None:
            return False, {'x': 0.5, 'y': 0.5}

        ret, frame = self.get_frame()
        if not ret or frame is None:
            return False, {'x': 0.5, 'y': 0.5}

        # Run Inference
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
            # PID Update
            self.pan -= pid_X.update(cx)
            self.tilt += pid_Y.update(cy)
            self.move_head(self.pan, self.tilt)
            
            locked = abs(cx - CENTER_X) < 40 and abs(cy - CENTER_Y) < 40
            return locked, {'x': cx/IMG_W, 'y': cy/IMG_H}
            
        return False, {'x': 0.5, 'y': 0.5}