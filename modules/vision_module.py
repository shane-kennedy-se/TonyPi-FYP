import os
import time
import sys
from collections import deque

# Handle missing vision modules gracefully
try:
    import cv2
    CV2_AVAILABLE = True
except ImportError:
    print("[WARNING] OpenCV (cv2) not available. Vision features disabled.")
    CV2_AVAILABLE = False
    cv2 = None

try:
    from ultralytics import YOLO
    YOLO_AVAILABLE = True
except ImportError:
    print("[WARNING] YOLO (ultralytics) not available. Object detection disabled.")
    YOLO_AVAILABLE = False
    YOLO = None

# Action imports - handle missing modules gracefully for development
try:
    import hiwonder.ActionGroupControl as AGC
    import hiwonder.Controller as Controller
    import hiwonder.ros_robot_controller_sdk as rrc
    from gpiozero import Button
    HARDWARE_AVAILABLE = True
except ImportError:
    print("[WARNING] Hardware modules not available. Running in simulation mode.")
    HARDWARE_AVAILABLE = False
    
    # Mock classes for development
    class MockAGC:
        @staticmethod
        def runActionGroup(action):
            print(f"[SIMULATION] Running action group: {action}")
    
    class MockController:
        def __init__(self, board):
            pass
        def set_bus_servo_deviation(self, servo, deviation):
            print(f"[SIMULATION] Setting servo {servo} to deviation {deviation}")
    
    class MockBoard:
        pass
    
    class MockButton:
        def __init__(self, pin, pull_up=True):
            pass
        def when_pressed(self, callback):
            pass
    
    AGC = MockAGC()
    Controller = MockController
    rrc = type('MockRRC', (), {'Board': MockBoard})()
    Button = MockButton

# CONFIGURATION
CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
BASE_DIR = os.path.dirname(CURRENT_DIR) 
MODEL_PATH = os.path.join(BASE_DIR, "resources", "models", "cardboard_v1.pt")

# --- TUNING SETTINGS ---
CONFIDENCE_THRESHOLD = 0.25 
LOCK_ENTER_TOLERANCE = 80   # <--- WIDENED (Was 60) to catch it easier
LOCK_EXIT_TOLERANCE = 120   # <--- WIDENED (Was 100)
MAX_MISSED_FRAMES = 10
SMOOTHING_BUFFER = 5

class VisionController:
    def __init__(self):
        print(f"[Vision] Loading YOLO model from: {MODEL_PATH}")
        if YOLO_AVAILABLE:
            try:
                self.model = YOLO(MODEL_PATH)
                self.class_names = self.model.names
                print(f"✅ Model Loaded Successfully!")
            except Exception as e:
                print(f"❌ CRITICAL ERROR: Could not load model. {e}")
                self.model = None
        else:
            print("[WARNING] YOLO not available - running without object detection")
            self.model = None

        self.last_action_time = 0
        self.is_locked = False 
        
        # Stability Variables
        self.missed_counter = 0
        self.last_valid_detection = None
        self.center_history = deque(maxlen=SMOOTHING_BUFFER)

        self.actions = {
            "cardboard": "Active",
            "box": "Active",
            "package": "Active"
        }

        # Action hardware setup - only if hardware is available
        if HARDWARE_AVAILABLE:
            self.rrc_board = rrc.Board()
            self.Board = Controller.Controller(self.rrc_board)
            self.agc = AGC.ActionGroupControl()
            
            # Touch sensor setup
            TOUCH_PIN = 18
            self.touch = Button(TOUCH_PIN, pull_up=True)
            self.emergency_stop = False
            
            def on_touch_detected():
                self.emergency_stop = True
                print("\n⚠ TOUCH DETECTED! EMERGENCY STOP ACTIVATED ⚠")
                self.agc.stopAll()
            
            self.touch.when_pressed = on_touch_detected
        else:
            self.rrc_board = None
            self.Board = MockController(None)
            self.agc = MockAGC()
            self.touch = None
            self.emergency_stop = False

    def reset(self):
        """
        Clears all memory. Call this when starting a new voice command.
        """
        self.is_locked = False
        self.missed_counter = 0
        self.last_valid_detection = None
        self.center_history.clear()
        print("[Vision] Logic Reset.")

    def detect(self, frame):
        if not self.model or not YOLO_AVAILABLE:
            print("[WARNING] Object detection not available")
            return None, 0, None, 0

        results = self.model(frame, verbose=False, conf=CONFIDENCE_THRESHOLD)
        
        raw_det = None
        max_conf = 0
        
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

        if raw_det:
            self.missed_counter = 0
            label, conf, box, cx = raw_det
            self.center_history.append(cx)
            smoothed_cx = int(sum(self.center_history) / len(self.center_history))
            self.last_valid_detection = (label, conf, box, smoothed_cx)
            return self.last_valid_detection
        else:
            if self.missed_counter < MAX_MISSED_FRAMES and self.last_valid_detection is not None:
                self.missed_counter += 1
                return self.last_valid_detection
            else:
                self.center_history.clear()
                self.is_locked = False
                return None

    def get_navigation_command(self, center_x, frame_width):
        screen_center = frame_width // 2
        error = center_x - screen_center
        limit = LOCK_EXIT_TOLERANCE if self.is_locked else LOCK_ENTER_TOLERANCE

        if abs(error) < limit:
            self.is_locked = True
            return "LOCKED", error
        else:
            self.is_locked = False
            return ("TURN_LEFT" if error < 0 else "TURN_RIGHT"), error

    def run_action(self, label):
        if label == "Peeling":
            return self.run_diecut_peeling()
        elif label == "Insert Label":
            return self.run_label_insertion()
        elif label == "Transport":
            return self.run_transport_box()
        elif label == "Flip":
            return self.run_sheet_flip_over()
        else:
            print(f"[Vision] Unknown action: {label}")
            return False

    def run_diecut_peeling(self):
        print("=== TonyPi Pro: Diecut Peeling Sequence ===")
        time.sleep(2)
        
        try:
            # Step 1: Pick up the diecut
            if self.emergency_stop: return False
            print("Step 1: Picking up diecut...")
            self.Board.set_bus_servo_deviation(17, 0)
            AGC.runActionGroup("PickUpDiecut1")
            time.sleep(0.5)

            # Step 2: Perform diecut peeling
            if self.emergency_stop: return False
            print("Step 2a: Performing diecut peeling (left hand)...")
            self.Board.set_bus_servo_deviation(17, -50)
            AGC.runActionGroup("DoDiecutLeftHand")
            time.sleep(0.5)

            if self.emergency_stop: return False
            print("Step 2b: Performing diecut peeling (right hand)...")
            self.Board.set_bus_servo_deviation(17, 0)
            AGC.runActionGroup("DoDiecutRightHand")
            time.sleep(0.5)

            # Step 3: Put down the diecut
            if self.emergency_stop: return False
            print("Step 3: Placing diecut down...")
            self.Board.set_bus_servo_deviation(17, 0)
            AGC.runActionGroup("PutDownDiecut1")
            time.sleep(0.5)

            print("=== Task Complete ===")
            return True

        except Exception as e:
            print(f"[ERROR] Diecut Peeling failed: {e}")
            return False

    def run_label_insertion(self):
        STEP_DELAY = 2
        print("=== TonyPi Pro: Label Insertion Sequence ===")
        time.sleep(2)

        try:
            # Step 1: Grab the label
            if self.emergency_stop: return False
            print("Step 1: Grabbing label...")
            AGC.runActionGroup("GrabLabel")
            time.sleep(STEP_DELAY)

            # Step 2: Lift label for insertion
            if self.emergency_stop: return False
            print("Step 2: Lifting label for insertion...")
            AGC.runActionGroup("LiftLabelInsertion")
            time.sleep(STEP_DELAY)

            # Step 3: Put label
            if self.emergency_stop: return False
            print("Step 3: Placing label...")
            AGC.runActionGroup("PutLabel")
            time.sleep(STEP_DELAY)

            # Step 4: Put down label and stand up
            if self.emergency_stop: return False
            print("Step 4: Putting down label and standing up...")
            AGC.runActionGroup("PutDownLabelAndStandUp")
            time.sleep(STEP_DELAY)

            print("=== Task Complete ===")
            return True

        except Exception as e:
            print(f"[ERROR] Label Insertion failed: {e}")
            return False

    def run_transport_box(self):
        steps_to_take = 5  # Default steps
        print(f"=== TonyPi Pro: Transport Box Sequence ({steps_to_take} steps) ===")
        time.sleep(2)

        try:
            # Step 1: Pick up the object
            if self.emergency_stop: return False
            print("Step 1: Picking up object...")
            AGC.runActionGroup("PickUp")
            time.sleep(0.5)

            # Step 2: Walk forward
            if self.emergency_stop: return False
            print("Step 2: Walking forward...")
            for i in range(steps_to_take):
                if self.emergency_stop: return False
                print(f"  Walking step {i+1}/{steps_to_take}...")
                AGC.runActionGroup("WalkOneStep")
                time.sleep(0.5)

            # Step 3: Put down the object
            if self.emergency_stop: return False
            print("Step 3: Placing object down...")
            AGC.runActionGroup("PutDown")

            print("=== Task Complete ===")
            return True

        except Exception as e:
            print(f"[ERROR] Transport Box failed: {e}")
            return False

    def run_sheet_flip_over(self):
        print("=== TonyPi Pro: Sheet Flip Sequence ===")
        time.sleep(2)

        try:
            # Step 1: Grab sheet
            if self.emergency_stop: return False
            print("Step 1: Grabbing sheet...")
            self.Board.set_bus_servo_deviation(16, 57)
            self.Board.set_bus_servo_deviation(18, 0)
            self.agc.runActionGroup("GrabSheet")
            time.sleep(1)

            # Step 2: Pass sheet
            if self.emergency_stop: return False
            print("Step 2: Passing sheet...")
            self.Board.set_bus_servo_deviation(16, 57)
            self.Board.set_bus_servo_deviation(18, 0)
            self.agc.runActionGroup("PassSheet")
            time.sleep(1)

            # Step 3: Change hand (flip sheet)
            if self.emergency_stop: return False
            print("Step 3: Changing hand (flip sheet)...")
            self.Board.set_bus_servo_deviation(16, -34)
            self.Board.set_bus_servo_deviation(18, -125)
            self.agc.runActionGroup("ChangeHandSheet")
            time.sleep(1)

            # Step 4: Put down sheet
            if self.emergency_stop: return False
            print("Step 4: Putting down sheet...")
            self.Board.set_bus_servo_deviation(18, 0)
            self.agc.runActionGroup("PutDownSheet")
            self.Board.set_bus_servo_deviation(16, 0)
            time.sleep(1)

            print("=== Task Complete ===")
            return True

        except Exception as e:
            print(f"[ERROR] Sheet Flip Over failed: {e}")
            return False