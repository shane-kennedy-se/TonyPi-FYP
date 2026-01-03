#!/usr/bin/python3
import sys
import time
import os

# --- PATHS ---
sys.path.append('/home/pi/TonyPi/HiwonderSDK')
sys.path.append('/home/pi/FYP_Robot')

from modules.voice_module import WonderEcho, speak
from modules.vision_detector import VisionDetector
import hiwonder.ActionGroupControl as AGC

print("--- TONY PI STARTING ---")
voice = WonderEcho()
vision = VisionDetector()

speak("System Online.")

try:
    while True:
        # 1. Listen for Command
        cmd = voice.get_command()
        
        if cmd:
            print(f"[CMD] {cmd}")
            
            # 2. Handle Voice & Movement
            if cmd == 'Greeting': speak("Hello human.")
            elif cmd == 'Wake Up': speak("I am here.")
            elif cmd == 'Sleep': speak("Goodnight.")
            
            elif cmd == 'Forward': AGC.runActionGroup('go_forward')
            elif cmd == 'Back': AGC.runActionGroup('back')
            elif cmd == 'Turn Left': AGC.runActionGroup('turn_left')
            elif cmd == 'Turn Right': AGC.runActionGroup('turn_right')
            elif cmd == 'Stop': 
                AGC.stopAction()
                speak("Stopped.")
            
            # 3. Handle Visual Task (Simple Pickup)
            elif cmd == 'Pick Up':
                speak("Scanning for target.")
                vision.move_head(1500, 1500)
                start_t = time.time()
                
                found = False
                while time.time() - start_t < 10:
                    locked, coords = vision.track_object()
                    if locked:
                        speak("Target locked.")
                        AGC.runActionGroup('PickUpDiecut') # Ensure this file exists!
                        speak("Done.")
                        found = True
                        break
                    time.sleep(0.02)
                
                if not found:
                    speak("I could not see anything.")

        time.sleep(0.02)

except KeyboardInterrupt:
    AGC.stopAction()