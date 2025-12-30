import cv2
import time
import os
import sys
import numpy as np
from ultralytics import YOLO

# --- FORCE RELEASE CAMERA ---
# This command stops the factory app from hogging the camera
os.system("sudo systemctl stop mjpg_streamer")
time.sleep(1) # Wait for it to let go

# --- CONFIG ---
IMG_W, IMG_H = 640, 480
CENTER_X = IMG_W // 2
CENTER_Y = IMG_H // 2
MODEL_PATH = "/home/pi/FYP_Robot/modules/cardboard_v1.pt"

# --- FACTORY ROBOT SDK ---
sys.path.append('/home/pi/TonyPi/HiwonderSDK')
try:
    import hiwonder.ros_robot_controller_sdk as rrc
    from hiwonder.Controller import Controller
    board = rrc.Board()
    ctl = Controller(board)
    print("[VISION] Real Robot Hardware Connected.")
except:
    print("[VISION] WARNING: Using Simulation Mode (No Servos)")
    ctl = None

class VisionDetector:
    def __init__(self):
        # 1. Load Model
        if os.path.exists(MODEL_PATH):
            self.model = YOLO(MODEL_PATH)
            print("[VISION] YOLO Loaded.")
        else:
            print(f"[VISION ERROR] Model not found at {MODEL_PATH}")
            self.model = None

        # 2. Connect to Camera (Robust Loop)
        self.camera = None
        # Try index 0, then 1, then -1 to find ANY working camera
        for idx in [0, 1, -1]:
            print(f"[VISION] Trying Camera Index {idx}...")
            cap = cv2.VideoCapture(idx)
            if cap.isOpened():
                self.camera = cap
                print(f"[VISION] Success! Connected to Camera {idx}")
                break
            cap.release()
        
        if not self.camera:
            print("[VISION CRITICAL] No Camera Found. Is the USB cable loose?")

        # 3. Init Head Position
        self.pan = 1500
        self.tilt = 1500
        self.move_head(1500, 1500)

    def move_head(self, pan, tilt):
        self.pan = max(500, min(2500, int(pan)))
        self.tilt = max(1200, min(2500, int(tilt)))
        if ctl:
            ctl.set_pwm_servo_pulse(1, self.pan, 20)
            ctl.set_pwm_servo_pulse(2, self.tilt, 20)

    def track_object(self):
        if not self.camera or not self.camera.isOpened():
            return False, None

        ret, frame = self.camera.read()
        if not ret: return False, None

        # Inference
        if self.model:
            results = self.model(frame, stream=True, verbose=False, conf=0.5)
            target = None
            
            for r in results:
                if len(r.boxes) > 0:
                    # Find the box with highest confidence
                    box = r.boxes[0]
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                    cx, cy = int((x1+x2)/2), int((y1+y2)/2)
                    target = (cx, cy)
                    break
            
            if target:
                cx, cy = target
                # Simple P-Controller for tracking
                error_x = CENTER_X - cx
                error_y = CENTER_Y - cy
                
                # Update servos (Note: '-' signs depend on servo orientation)
                self.pan += int(error_x * 0.08)
                self.tilt += int(error_y * 0.08)
                self.move_head(self.pan, self.tilt)
                
                # Check if locked (near center)
                locked = abs(error_x) < 50 and abs(error_y) < 50
                return locked, {'y': cy / IMG_H}

        return False, None