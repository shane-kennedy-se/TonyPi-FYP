#!/usr/bin/python3
import cv2
import time
import sys
import threading
import subprocess
import os

# --- HARDWARE IMPORTS ---
import hiwonder.Camera as Camera
from modules import voice_module
from modules import vision_module
from modules import light_sensor
from modules import ultrasonic_sensor  

# --- CONFIGURATION ---
FRAME_WIDTH = 640
FRAME_HEIGHT = 480

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
    voice = voice_module.WonderEcho()
    vision = vision_module.VisionController()
    
    # Initialize Light Sensor on Pin 24
    sensor = light_sensor.LightSensor(pin=24)
    
    # Initialize Ultrasonic Sensor for obstacle detection
    ultrasonic = ultrasonic_sensor.UltrasonicSensor()

    # 2. Open Camera
    print("📷 Opening Hiwonder Camera...")
    try:
        cap = Camera.Camera()
        cap.camera_open()
    except Exception as e:
        print(f"❌ CRITICAL: Could not open camera. {e}")
        return

    # 3. Start Vision Brain
    ai_thread = threading.Thread(target=inference_worker, args=(vision,))
    ai_thread.daemon = True
    ai_thread.start()

    current_state = STATE_IDLE
    current_task = None
    was_dark_last_frame = False
    
    # Initial Voice Check
    voice_module.speak("System online.")
    print("✅ System Ready.")

    try:
        while True:
            # A. UPDATE CAMERA
            ret, frame = cap.read()
            if not ret: 
                time.sleep(0.01)
                continue
            with frame_lock: latest_frame = frame

            # ==========================================
            # 🚨 SAFETY CHECK: LIGHT SENSOR
            # ==========================================
            # Check if it is dark using your class
            is_dark_now = sensor.is_dark()
            
            if is_dark_now:
                # 🛑 DANGER: TOO DARK!
                if not was_dark_last_frame:
                    voice_module.speak("Too dark. Stopping now.")
                
                # If we were doing something, STOP and SPEAK.
                if current_state != STATE_IDLE:
                    print("⚠️ DARKNESS DETECTED! ABORTING ACTION!")
                    current_state = STATE_IDLE
                    current_task = None
                    vision.reset() # Reset vision memory
                
                # Visual Warning on Screen
                cv2.rectangle(frame, (0,0), (FRAME_WIDTH, FRAME_HEIGHT), (0,0,255), 5)
                cv2.putText(frame, "⚠️ TOO DARK - STOPPED", (150, 240), 
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 3)
                
                was_dark_last_frame = True
                
                # Skip the rest of the loop (Don't listen or look)
                cv2.imshow("TonyPi", frame)
                if cv2.waitKey(1) & 0xFF == ord('q'): break
                continue
            
            # If light just came back on, notify user ONE time
            if was_dark_last_frame and not is_dark_now:
                voice_module.speak("Light levels normal. System ready.")
                was_dark_last_frame = False

            # ==========================================
            # 🚨 OBSTACLE DETECTION: ULTRASONIC SENSOR
            # ==========================================
            if current_state != STATE_IDLE:
                distance = ultrasonic.get_distance()
                if distance is not None and ultrasonic.is_obstacle_detected():
                    print(f"⚠️ OBSTACLE DETECTED at {distance}cm! ABORTING ACTION!")
                    voice_module.speak(f"Obstacle detected. Stopping.")
                    
                    # Stop current action and return to idle
                    current_state = STATE_IDLE
                    current_task = None
                    vision.reset()
                    
                    # Visual warning
                    cv2.rectangle(frame, (0,0), (FRAME_WIDTH, FRAME_HEIGHT), (0, 165, 255), 5)
                    cv2.putText(frame, f"🚨 OBSTACLE: {distance}cm", (150, 240), 
                               cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 165, 255), 3)

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
                        vision.reset()
                    
                    # --- UPDATED COMMAND LIST HERE ---
                    elif cmd in ["Peeling", "Insert Label", "Flip", "Transport", "Pick Up Cardboard", "Transport Cardboard"]:
                        # Final check before starting
                        if not sensor.is_dark():
                            # ====================================================
                            # 📝 LOGIC STEP 1: VOICE TRIGGER
                            # ====================================================
                            # We received a valid command (e.g., "Pick Up Cardboard").
                            # 1. Store the command as 'current_task'.
                            # 2. Switch state to SEARCHING.
                            # 3. The robot will NOT act yet; it must find the box first.
                            current_task = cmd
                            voice_module.speak(f"Starting {cmd}. Searching.")
                            current_state = STATE_SEARCHING
                            vision.reset()  # Clear old vision memory
                        else:
                            voice_module.speak("Cannot start. It is too dark.")

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
                        
                        # ====================================================
                        # 📝 LOGIC STEP 2: VISION LOCK
                        # ====================================================
                        # The vision system has successfully centered the cardboard.
                        # Now we switch to ACTING to perform the physical task.
                        current_state = STATE_ACTING
                        
                    elif nav_cmd == "TURN_LEFT":
                        cv2.arrowedLine(frame, (320, 240), (270, 240), (255, 255, 0), 3)
                    elif nav_cmd == "TURN_RIGHT":
                        cv2.arrowedLine(frame, (320, 240), (370, 240), (255, 255, 0), 3)
                else:
                    cv2.putText(frame, "Scanning...", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 165, 255), 2)

            elif current_state == STATE_ACTING:
                cv2.putText(frame, f"TASK: {current_task}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                
                # ====================================================
                # 📝 LOGIC STEP 3: EXECUTE ACTION
                # ====================================================
                # We now run the specific script for the task we saved in Step 1.
                print(f"[Main] Running action: {current_task}")
                success = vision.run_action(current_task)
                
                if not success:
                    time.sleep(3) 
                
                voice_module.speak(f"{current_task} complete.")
                current_state = STATE_IDLE
                current_task = None

            cv2.imshow("TonyPi", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'): break

    except KeyboardInterrupt:
        print("Stopping...")
    finally:
        running = False
        sensor.cleanup()
        ultrasonic.cleanup()
        cap.camera_close()
        cv2.destroyAllWindows()
        ai_thread.join()

if __name__ == "__main__":
    main()