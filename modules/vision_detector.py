import cv2
import os
import sys
import time
from ultralytics import YOLO

# --- FORCE RELEASE CAMERA ---
os.system("sudo systemctl stop mjpg_streamer")
time.sleep(1)

# --- ROBOT SDK ---
sys.path.append('/home/pi/TonyPi/HiwonderSDK')
try:
    import hiwonder.ros_robot_controller_sdk as rrc
    from hiwonder.Controller import Controller
    board = rrc.Board()
    ctl = Controller(board)
except:
    print("[VISION WARNING] Robot hardware not found (Sim Mode)")
    ctl = None

MODEL_PATH = "/home/pi/FYP_Robot/modules/cardboard_v1.pt"

class VisionDetector:
    def __init__(self):
        # 1. Load Model
        if os.path.exists(MODEL_PATH):
            self.model = YOLO(MODEL_PATH)
            print("[VISION] Model Loaded.")
        else:
            self.model = None
            print(f"[VISION ERROR] Model not found at {MODEL_PATH}")

        # 2. Connect Camera (Retry Loop)
        self.camera = None
        for i in [0, -1]:
            cap = cv2.VideoCapture(i)
            if cap.isOpened():
                self.camera = cap
                print(f"[VISION] Connected to Camera {i}")
                break
        
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
        if not self.camera: return False, None
        
        ret, frame = self.camera.read()
        if not ret: return False, None

        if self.model:
            results = self.model(frame, stream=True, verbose=False, conf=0.5)
            for r in results:
                if len(r.boxes) > 0:
                    box = r.boxes[0]
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                    cx, cy = int((x1+x2)/2), int((y1+y2)/2)
                    
                    # Tracking Logic
                    err_x = 320 - cx
                    err_y = 240 - cy
                    
                    self.pan += int(err_x * 0.05)
                    self.tilt += int(err_y * 0.05)
                    self.move_head(self.pan, self.tilt)
                    
                    locked = abs(err_x) < 40 and abs(err_y) < 40
                    return locked, {'y': cy/480}
                    
        return False, None