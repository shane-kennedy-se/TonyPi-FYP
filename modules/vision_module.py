import os
import time
import cv2
import sys
import subprocess
import numpy as np
from collections import deque
from ultralytics import YOLO

# CONFIGURATION
CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
BASE_DIR = os.path.dirname(CURRENT_DIR) 
MODEL_PATH = os.path.join(BASE_DIR, "resources", "models", "cardboard_v1.pt")
FUNCTIONS_DIR = os.path.join(BASE_DIR, "Functions")

# --- TUNING SETTINGS ---
CONFIDENCE_THRESHOLD = 0.4

# 1. HYSTERESIS (Sticky Lock)
LOCK_ENTER_TOLERANCE = 60   # Easier to get locked (wider)
LOCK_EXIT_TOLERANCE = 100   # Harder to lose lock (must move far away)

# 2. PERSISTENCE (Memory)
# If YOLO misses the object, we remember it for this many frames before giving up.
MAX_MISSED_FRAMES = 10      # Approx 0.3 seconds of memory

# 3. SMOOTHING (Jitter reduction)
# We average the last 5 center positions to stop the shaking.
SMOOTHING_BUFFER = 5

class VisionController:
    def __init__(self):
        print(f"[Vision] Loading YOLO model from: {MODEL_PATH}")
        try:
            self.model = YOLO(MODEL_PATH)
            self.class_names = self.model.names
            print(f"[Vision] Model Loaded. Classes: {self.class_names}")
        except Exception as e:
            print(f"[Vision] CRITICAL ERROR: Could not load model. {e}")
            self.model = None

        self.last_action_time = 0
        self.is_locked = False 
        
        # Stability Variables
        self.missed_counter = 0
        self.last_valid_detection = None
        self.center_history = deque(maxlen=SMOOTHING_BUFFER)

        self.actions = {
            "cardboard": "SheetFlipOver.py",
            "box": "SheetFlipOver.py"
        }

    def detect(self, frame):
        """
        Runs inference with Persistence and Smoothing.
        Returns: (label, confidence, box_coords, center_x)
        """
        if not self.model:
            return None, 0, None, 0

        results = self.model(frame, verbose=False, conf=CONFIDENCE_THRESHOLD)
        
        raw_det = None
        max_conf = 0
        
        # 1. Find best detection in current frame
        for r in results:
            for box in r.boxes:
                conf = float(box.conf[0])
                cls_id = int(box.cls[0])
                label = self.class_names[cls_id]
                
                if label.lower() in self.actions and conf > max_conf:
                    max_conf = conf
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    cx = int((x1 + x2) / 2)
                    raw_det = (label, conf, (x1, y1, x2, y2), cx)

        # 2. Apply Persistence Logic
        if raw_det:
            # We found it! Reset miss counter
            self.missed_counter = 0
            
            # Add center to history for smoothing
            label, conf, box, cx = raw_det
            self.center_history.append(cx)
            
            # Calculate smoothed center
            smoothed_cx = int(sum(self.center_history) / len(self.center_history))
            
            # Update the valid detection with the SMOOTHED center
            self.last_valid_detection = (label, conf, box, smoothed_cx)
            return self.last_valid_detection

        else:
            # We missed it this frame. Do we remember it?
            if self.missed_counter < MAX_MISSED_FRAMES and self.last_valid_detection is not None:
                self.missed_counter += 1
                # Return the OLD detection (Ghost mode)
                return self.last_valid_detection
            else:
                # We truly lost it
                self.center_history.clear()
                self.is_locked = False # Force unlock if lost
                return None

    def get_navigation_command(self, center_x, frame_width):
        """
        Returns a command with Hysteresis (Sticky Logic).
        """
        screen_center = frame_width // 2
        error = center_x - screen_center
        abs_error = abs(error)

        # Use sticky thresholds
        limit = LOCK_EXIT_TOLERANCE if self.is_locked else LOCK_ENTER_TOLERANCE

        if abs_error < limit:
            self.is_locked = True
            return "LOCKED", error
        else:
            self.is_locked = False
            if error < 0:
                return "TURN_LEFT", error
            else:
                return "TURN_RIGHT", error

    def run_action(self, label):
        ACTION_COOLDOWN = 5.0
        if time.time() - self.last_action_time < ACTION_COOLDOWN:
            return False

        script_name = self.actions.get(label.lower())
        if not script_name:
            return False

        print(f"[Vision] >>> STABLE LOCK! TRIGGERING: {script_name} <<<")
        self.last_action_time = time.time()
        return True