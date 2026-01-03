#!/usr/bin/python3
import cv2
import time
import sys
import threading
import subprocess
import os

# --- IMPORT MODULES ---
from modules import voice_module
from modules import vision_module

# --- CONFIGURATION ---
FRAME_WIDTH = 640
FRAME_HEIGHT = 480

#ROBOT STATES
STATE_IDLE = "IDLE"           # Waiting for voice command
STATE_SEARCHING = "SEARCHING" # Active, looking for cardboard
STATE_ACTING = "ACTING"       # Busy performing the task

# SHARED VARIABLES (Threading)
latest_frame = None
latest_result = None
running = True
frame_lock = threading.Lock()
result_lock = threading.Lock()

# --- VISION THREAD (The Brain) ---
def inference_worker(vision):
    """Background thread that runs YOLO constantly."""
    global latest_frame, latest_result, running
    
    print("🧠 AI Vision Thread Started...")
    
    while running:
        img_to_process = None
        with frame_lock:
            if latest_frame is not None:
                img_to_process = latest_frame.copy()
        
        if img_to_process is not None:
            # Run detection
            detection_data = vision.detect(img_to_process)
            
            with result_lock:
                latest_result = detection_data
        
        time.sleep(0.01) # Prevent CPU hogging

# --- MAIN CONTROL LOOP ---
def main():
    global latest_frame, running, latest_result
    
    print("------------------------------------------")
    print("      TONYPI ROBOT: MAIN CONTROLLER       ")
    print("      (Voice + Vision + Action)           ")
    print("------------------------------------------")

    # 1. Initialize Modules
    voice = voice_module.WonderEcho()
    vision = vision_module.VisionController()
    
    # 2. Initialize Camera
    cap = cv2.VideoCapture(0)
    cap.set(3, FRAME_WIDTH)
    cap.set(4, FRAME_HEIGHT)
    
    if not cap.isOpened():
        print("❌ CRITICAL: Camera failed to open.")
        return

    # 3. Start Vision Thread
    ai_thread = threading.Thread(target=inference_worker, args=(vision,))
    ai_thread.daemon = True
    ai_thread.start()

    # 4. Robot State
    current_state = STATE_IDLE
    current_task = None
    
    # Give the system a moment to warm up
    time.sleep(2)
    voice_module.speak("System online. Waiting for command.")

    print("✅ System Ready. Say a command!")

    try:
        while True:
            # --- A. UPDATE CAMERA FEED ---
            ret, frame = cap.read()
            if not ret: break
            
            with frame_lock:
                latest_frame = frame

            # --- B. CHECK VOICE COMMANDS (Non-Blocking) ---
            # We only listen for new commands if we aren't busy acting
            if current_state != STATE_ACTING:
                cmd = voice.get_command()
                
                if cmd:
                    print(f"🎤 Voice Command: {cmd}")
                    
                    if cmd == "Wake Up":
                        voice_module.speak("I am listening.")
                        current_state = STATE_IDLE
                    
                    elif cmd == "Stop":
                        voice_module.speak("Stopping immediately.")
                        current_state = STATE_IDLE
                        current_task = None
                        
                    # TASK COMMANDS (Triggers Vision)
                    elif cmd in ["Peeling", "Insert Label", "Transport", "Flip"]:
                        current_task = cmd
                        voice_module.speak(f"Okay, starting {cmd}. Looking for cardboard.")
                        current_state = STATE_SEARCHING
                        # Reset vision lock memory for a fresh search
                        vision.is_locked = False 
                        vision.center_history.clear()

            # --- C. MAIN LOGIC (Based on State) ---
            
            # STATE: IDLE
            if current_state == STATE_IDLE:
                cv2.putText(frame, "STATUS: IDLE (Waiting for Voice)", (10, 30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (200, 200, 200), 2)

            # STATE: SEARCHING
            elif current_state == STATE_SEARCHING:
                # Retrieve latest vision result
                current_det = None
                with result_lock:
                    current_det = latest_result
                
                if current_det:
                    label, conf, box, cx = current_det
                    x1, y1, x2, y2 = box
                    
                    # Calculate Navigation
                    nav_cmd, error = vision.get_navigation_command(cx, FRAME_WIDTH)
                    
                    # Visual Feedback
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.putText(frame, f"TARGET: {label}", (x1, y1-10), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                    # LOGIC: LOCKING
                    if nav_cmd == "LOCKED":
                        color = (0, 0, 255)
                        cv2.putText(frame, "!!! TARGET LOCKED !!!", (cx - 80, y1 - 30), 
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
                        
                        # Trigger Action Sequence
                        voice_module.speak("Target locked.")
                        current_state = STATE_ACTING
                        
                    elif nav_cmd == "TURN_LEFT":
                        cv2.arrowedLine(frame, (320, 240), (270, 240), (255, 255, 0), 3)
                        # TODO: Add real robot turn command here (e.g. chassis.turn_left())
                        
                    elif nav_cmd == "TURN_RIGHT":
                        cv2.arrowedLine(frame, (320, 240), (370, 240), (255, 255, 0), 3)
                        # TODO: Add real robot turn command here
                
                else:
                    cv2.putText(frame, "Scanning...", (10, 60), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 165, 255), 2)
                    # TODO: Add logic to spin robot slowly if nothing found

            # STATE: ACTING (The "Action" Phase)
            elif current_state == STATE_ACTING:
                cv2.putText(frame, f"EXECUTING: {current_task}", (10, 30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                
                # Here we simulate the action time (Arms moving, etc.)
                # In the future, you will run: subprocess.run(["python3", "Functions/SheetFlip.py"])
                
                print(f"[Main] Executing task: {current_task}")
                
                # 1. Speak status
                if current_task == "Peeling":
                    voice_module.speak("Performing die-cut peeling.")
                elif current_task == "Transport":
                    voice_module.speak("Transporting cardboard now.")
                
                # 2. Wait (Simulate work)
                time.sleep(3) 
                
                # 3. Finish
                voice_module.speak(f"{current_task} complete.")
                current_state = STATE_IDLE
                current_task = None

            # --- D. DISPLAY HUD ---
            cv2.imshow("TonyPi Robot View", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    except KeyboardInterrupt:
        print("Stopping...")
    
    finally:
        running = False
        cap.release()
        cv2.destroyAllWindows()
        ai_thread.join()
        print("Program Exited.")

if __name__ == "__main__":
    main()