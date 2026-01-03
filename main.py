#!/usr/bin/python3
import sys
import time
import os

# 1. FIX AUDIO PERMISSIONS (Essential for Piper)
os.environ["XDG_RUNTIME_DIR"] = "/run/user/1000"

# 2. PATH SETUP
sys.path.append('/home/pi/TonyPi/HiwonderSDK')
sys.path.append('/home/pi/FYP_Robot')

# 3. IMPORT MODULES
from modules.voice_module import WonderEcho, speak
from modules.vision_detector import VisionDetector
from modules.light_sensor import LightSensor
import hiwonder.ActionGroupControl as AGC

print("--- TONY PI ONLINE ---")

# 4. INIT MODULES
try:
    voice = WonderEcho()
    vision = VisionDetector() # Uses your working camera logic now
    light = LightSensor(pin=24)
    speak("System Online.")
except Exception as e:
    print(f"[INIT ERROR] {e}")

# 5. MAIN LOOP
try:
    while True:
        # --- A. SAFETY (Light Sensor) ---
        if light.is_dark():
            AGC.stopAction()
            speak("It is too dark.")
            time.sleep(3)
            continue

        # --- B. LISTENING ---
        cmd = voice.get_command()
        
        if cmd:
            print(f"[CMD]: {cmd}")
            
            # --- PERSONALITY RESPONSES (From your Sheet) ---
            if cmd == 'Wake Up':      speak("Yes boss?")
            elif cmd == 'Greeting':   speak("Welcome.")
            elif cmd == 'Sleep':      speak("I'm going to take a break.")
            
            # --- MOVEMENT ---
            elif cmd == 'Forward':
                speak("Stepping forward.")
                AGC.runActionGroup('go_forward')
            
            elif cmd == 'Back':
                speak("Stepping back.")
                AGC.runActionGroup('back')
            
            elif cmd == 'Turn Left':
                speak("Turning left.")
                AGC.runActionGroup('turn_left')
            
            elif cmd == 'Turn Right':
                speak("Turning right.")
                AGC.runActionGroup('turn_right')

            elif cmd == 'Stop':
                AGC.stopAction()
                speak("Stopping immediately.")

            # --- TASKS WITH VISION ---
            elif cmd == 'Pick Up':
                speak("I am picking it up.")
                
                # Scan Phase
                vision.move_head(1500, 1500)
                start_t = time.time()
                found = False
                
                while time.time() - start_t < 8:
                    locked, coords = vision.track_object()
                    if locked:
                        speak("Target locked.")
                        # AGC.runActionGroup('PickUpDiecut') # Enable real movement here
                        found = True
                        break
                    time.sleep(0.02)
                
                if not found:
                    speak("I cannot see it.")
            
            elif cmd == 'Transport': speak("I will transport it.")
            elif cmd == 'Peeling': speak("Okay, peeling now.")
            elif cmd == 'Flip': speak("Okay, flipping now.")

        time.sleep(0.02)

except KeyboardInterrupt:
    AGC.stopAction()
    print("Exiting...")