import os
import time
import cv2
import sys
import subprocess
from ultralytics import YOLO

# CONFIGURATION
CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
BASE_DIR = os.path.dirname(CURRENT_DIR) 
MODEL_PATH = os.path.join(BASE_DIR, "resources", "models", "cardboard_v1.pt")
FUNCTIONS_DIR = os.path.join(BASE_DIR, "Functions")

# Logic Settings
CONFIDENCE_THRESHOLD = 0.4  
CENTER_TOLERANCE = 50       
ACTION_COOLDOWN = 5.0       

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
        
        # Mapping actions (Disabled for now, but good to keep for reference)
        self.actions = {
            "cardboard": "SheetFlipOver.py",
            "box": "SheetFlipOver.py"
        }

    def detect(self, frame):
        """
        Runs inference. Returns: (label, confidence, box_coords, center_x)
        """
        if not self.model:
            return None, 0, None, 0

        # Run YOLO inference
        results = self.model(frame, verbose=False, conf=CONFIDENCE_THRESHOLD)
        
        best_det = None
        max_conf = 0
        
        for r in results:
            for box in r.boxes:
                conf = float(box.conf[0])
                cls_id = int(box.cls[0])
                label = self.class_names[cls_id]
                
                # Check if it matches our target list
                if label.lower() in self.actions and conf > max_conf:
                    max_conf = conf
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    cx = int((x1 + x2) / 2)
                    best_det = (label, conf, (x1, y1, x2, y2), cx)

        return best_det

    def get_navigation_command(self, center_x, frame_width):
        """
        Returns a command to help the robot lock onto the target.
        """
        screen_center = frame_width // 2
        error = center_x - screen_center
        
        if abs(error) < CENTER_TOLERANCE:
            return "LOCKED", error
        elif error < 0:
            return "TURN_LEFT", error
        else:
            return "TURN_RIGHT", error

    def run_action(self, label):
        """
        MOCKED: Just prints that an action would happen.
        """
        if time.time() - self.last_action_time < ACTION_COOLDOWN:
            # print(f"[Vision] Cooldown active. Ignoring {label}.")
            return False

        script_name = self.actions.get(label.lower())
        if not script_name:
            return False

        # --- ACTION DISABLED FOR VISION TESTING ---
        print(f"[Vision] >>> LOCKED ON TARGET! (Action '{script_name}' skipped) <<<")
        
        # script_path = os.path.join(FUNCTIONS_DIR, script_name)
        # subprocess.run(["python3", script_path], check=False)
        # ------------------------------------------
        
        self.last_action_time = time.time()
        return True