#!/usr/bin/python3
import cv2
import time
import sys
import threading
import subprocess
import os

# --- HARDWARE IMPORTS ---
import hiwonder.Camera as Camera  # <--- FIXED: Use official Camera library
try:
    import RPi.GPIO as GPIO
except ImportError:
    GPIO = None

# --- IMPORT MODULES ---
from modules import voice_module
from modules import vision_module

# --- CONFIGURATION ---
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
LIGHT_SENSOR_PIN = 11  # CHECK THIS PIN ON YOUR ROBOT!

# ROBOT STATES
STATE_IDLE = "IDLE"           
STATE_SEARCHING = "SEARCHING" 
STATE_ACTING = "ACTING"       

# THREADING SHARED VARS
latest_frame = None
latest_result = None
running = True
frame_lock = threading.Lock()
result_lock = threading.Lock()

# ==========================================
# 💡 LIGHT SENSOR SETUP
# ==========================================
def setup_gpio():
    if GPIO:
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(LIGHT_SENSOR_PIN, GPIO.IN)

def check_light_sensor():
    if GPIO is None: return True
    # Read Pin: Assuming 1=Dark, 0=Light. Swap if needed!
    is_dark = GPIO.input(LIGHT_SENSOR_PIN)
    return not is_dark 

# ==========================================
# 🧠 VISION THREAD
# ==========================================
def inference_worker(vision):
    global latest_frame, latest_result, running
    print("🧠 AI Vision Thread Started...")
    while running:
        img_to_process = None
        with frame_lock:
            if latest_frame is not None:
                img_to_process = latest_frame.copy()
        
        if img_to_process is not None:
            detection_data = vision.detect(img_to_process)
            with result_lock:
                latest_result = detection_data
        time.sleep(0.01)

# ==========================================
# 🎮 MAIN CONTROLLER
# ==========================================
def main():
    global latest_frame, running, latest_result
    print("------------------------------------------")
    print("      TONYPI ROBOT: MAIN CONTROLLER       ")
    print("------------------------------------------")

    # 1. Setup Hardware
    setup_gpio()
    voice = voice_module.WonderEcho()
    vision = vision_module.VisionController()
    
    # --- CAMERA FIX START ---
    # We use the Hiwonder Camera class instead of cv2.VideoCapture(0)
    print("📷 Opening Hiwonder Camera...")
    try:
        cap = Camera.Camera()
        cap.camera_open()
    except Exception as e:
        print(f"❌ CRITICAL: Could not open camera. {e}")
        return
    # --- CAMERA FIX END ---

    # 2. Start Vision Brain
    ai_thread = threading.Thread(target=inference_worker, args=(vision,))
    ai_thread.daemon = True
    ai_thread.start()

    current_state = STATE_IDLE
    current_task = None
    was_dark_last_frame = False
    
    voice_module.speak("System online.")
    print("✅ System Ready.")

    try:
        while True:
            # A. UPDATE CAMERA
            ret, frame = cap.read()
            if not ret: 
                # If frame is bad, wait a tiny bit and try again (don't crash)
                time.sleep(0.01)
                continue
                
            with frame_lock: latest_frame = frame

            # ==========================================
            # 🚨 SAFETY CHECK: LIGHT SENSOR
            # ==========================================
            is_light_safe = check_light_sensor()
            
            if not is_light_safe:
                if current_state != STATE_IDLE:
                    print("⚠️ DARKNESS DETECTED! ABORTING ACTION!")
                    voice_module.speak("Too dark. Stopping.")
                    current_state = STATE_IDLE
                    current_task = None
                    vision.is_locked = False
                
                # Visual Warning
                cv2.rectangle(frame, (0,0), (FRAME_WIDTH, FRAME_HEIGHT), (0,0,255), 5)
                cv2.putText(frame, "TOO DARK", (150, 240), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 3)
                was_dark_last_frame = True
                
                cv2.imshow("TonyPi", frame)
                if cv2.waitKey(1) & 0xFF == ord('q'): break
                continue
            
            if was_dark_last_frame and is_light_safe:
                voice_module.speak("Light normal.")
                was_dark_last_frame = False

            # ==========================================
            # 🎤 VOICE COMMANDS
            # ==========================================
            if current_state != STATE_ACTING:
                cmd = voice.get_command()
                if cmd:
                    print(f"🎤 Command: {cmd}")
                    
                    if cmd == "Wake Up":
                        voice_module.speak("Listening.")
                        current_state = STATE_IDLE
                    
                    elif cmd == "Stop":
                        voice_module.speak("Stopping.")
                        current_state = STATE_IDLE
                        vision.is_locked = False
                    
                    elif cmd in ["Peeling", "Insert Label", "Transport", "Flip"]:
                        if check_light_sensor():
                            current_task = cmd
                            voice_module.speak(f"Starting {cmd}. Searching.")
                            current_state = STATE_SEARCHING
                            vision.reset()

            # ==========================================
            # 🤖 ROBOT LOGIC
            # ==========================================
            if current_state == STATE_IDLE:
                cv2.putText(frame, "IDLE", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

            elif current_state == STATE_SEARCHING:
                current_det = None
                with result_lock: current_det = latest_result
                
                if current_det:
                    label, conf, box, cx = current_det
                    x1, y1, x2, y2 = box
                    nav_cmd, error = vision.get_navigation_command(cx, FRAME_WIDTH)
                    
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

                    if nav_cmd == "LOCKED":
                        cv2.putText(frame, "LOCKED", (x1, y1-30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255), 2)
                        voice_module.speak("Target locked.")
                        current_state = STATE_ACTING
                        
                    elif nav_cmd == "TURN_LEFT":
                        cv2.arrowedLine(frame, (320, 240), (270, 240), (255, 255, 0), 3)
                    elif nav_cmd == "TURN_RIGHT":
                        cv2.arrowedLine(frame, (320, 240), (370, 240), (255, 255, 0), 3)
                else:
                    cv2.putText(frame, "Scanning...", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 165, 255), 2)

            elif current_state == STATE_ACTING:
                cv2.putText(frame, f"TASK: {current_task}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                
                print(f"[Main] Running action: {current_task}")
                success = vision.run_action(current_task)
                
                if not success:
                    time.sleep(3) # Simulate action
                
                voice_module.speak(f"{current_task} complete.")
                current_state = STATE_IDLE
                current_task = None

            cv2.imshow("TonyPi", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'): break

    except KeyboardInterrupt:
        print("Stopping...")
    finally:
        running = False
        if GPIO: GPIO.cleanup()
        cap.camera_close() # Use the proper close command
        cv2.destroyAllWindows()
        ai_thread.join()

if __name__ == "__main__":
    main()