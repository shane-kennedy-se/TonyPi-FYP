import cv2
import os
import time
import sys
from ultralytics import YOLO

# --- PATHS (Matches your uploaded file structure) ---
BASE_DIR = "/home/pi/FYP_Robot"
MODEL_PATH = os.path.join(BASE_DIR, "resources", "models", "cardboard_v1.pt")

# --- ROBOT HARDWARE (Real SDK) ---
sys.path.append('/home/pi/TonyPi/HiwonderSDK')
try:
    import hiwonder.ros_robot_controller_sdk as rrc
    from hiwonder.Controller import Controller
    board = rrc.Board()
    ctl = Controller(board)
except:
    ctl = None

class VisionDetector:
    def __init__(self):
        # 1. LOAD MODEL (Exact logic from your file)
        print(f"[VISION] Loading model: {MODEL_PATH}")
        if os.path.exists(MODEL_PATH):
            try:
                self.model = YOLO(MODEL_PATH)
                print("[VISION] YOLO Model Loaded successfully.")
            except Exception as e:
                print(f"[VISION ERROR] Model load failed: {e}")
                self.model = None
        else:
            print(f"[VISION ERROR] Model file missing at {MODEL_PATH}")
            self.model = None

        # 2. CONNECT CAMERA (Exact logic from your file)
        self.camera = cv2.VideoCapture(0)
        self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        try:
            self.camera.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        except: pass

        if self.camera.isOpened():
            print("[VISION] Camera Index 0 Opened Successfully.")
        else:
            print("[VISION CRITICAL] Could not open Camera 0. Run 'sudo fuser -k -v /dev/video0'")

        # 3. INIT HEAD POSITION
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
        """
        Returns: (locked_status, coords)
        locked_status: True if object is centered
        coords: {'y': normalized_height} for distance adjustment
        """
        if not self.camera or not self.camera.isOpened():
            return False, None

        ret, frame = self.camera.read()
        if not ret: return False, None

        if self.model:
            # Inference (Your settings: conf=0.1, verbose=False)
            results = self.model(frame, stream=True, verbose=False, conf=0.1)
            
            target = None
            max_conf = 0.0

            for r in results:
                for box in r.boxes:
                    # Find the most confident cardboard
                    conf = float(box.conf[0])
                    if conf > max_conf:
                        x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                        cx, cy = int((x1+x2)/2), int((y1+y2)/2)
                        target = (cx, cy)
                        max_conf = conf

            if target:
                cx, cy = target
                # Simple Tracking Logic (Center the object)
                # Image is 640x480. Center is 320x240.
                err_x = 320 - cx
                err_y = 240 - cy
                
                # Proportional Control
                self.pan += int(err_x * 0.06)
                self.tilt += int(err_y * 0.06)
                self.move_head(self.pan, self.tilt)
                
                # Check if "Locked" (Close to center)
                locked = abs(err_x) < 50 and abs(err_y) < 50
                return locked, {'y': cy/480.0}
        
        return False, None