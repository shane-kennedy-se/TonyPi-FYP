#!/usr/bin/python3
import sys
import time
import os

# --- PATHS ---
sys.path.append('/home/pi/TonyPi/HiwonderSDK')
sys.path.append('/home/pi/FYP_Robot')

from modules.voice_module import WonderEcho, speak
import hiwonder.ActionGroupControl as AGC

print("--- TONYPI SYSTEM STARTING ---")
voice = WonderEcho()
speak("System Online.")

try:
    while True:
        cmd = voice.get_command()
        
        if cmd:
            print(f"[COMMAND RECEIVED]: {cmd}")
            
            # --- ACTION MAPPING ---
            if cmd == 'Greeting': 
                speak("Hello! I am Tony Pi.")
            
            elif cmd == 'Wake Up': 
                speak("Yes boss?")
            
            elif cmd == 'Sleep': 
                speak("Going to sleep.")
            
            elif cmd == 'Forward': 
                AGC.runActionGroup('go_forward')
            
            elif cmd == 'Back': 
                AGC.runActionGroup('back')
            
            elif cmd == 'Turn Left': 
                AGC.runActionGroup('turn_left')
            
            elif cmd == 'Turn Right': 
                AGC.runActionGroup('turn_right')
            
            elif cmd == 'Stop': 
                AGC.stopAction()
                speak("Stopped.")

            # --- COMPLEX TASKS (Placeholder for now) ---
            elif cmd == 'Pick Up':
                speak("Starting pick up sequence.")
                # run_visual_task("pickup") # We enable this later
            
            elif cmd == 'Insert Label':
                speak("Inserting label.")
            
            elif cmd == 'Peeling':
                speak("Starting peeling.")

        time.sleep(0.02)

except KeyboardInterrupt:
    AGC.stopAction()
    print("Exiting...")