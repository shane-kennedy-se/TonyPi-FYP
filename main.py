#!/usr/bin/python3
import sys
import time
import os

# --- PATHS ---
sys.path.append('/home/pi/TonyPi/HiwonderSDK')
sys.path.append('/home/pi/FYP_Robot')

from modules.voice_module import WonderEcho, speak
from modules.vision_detector import VisionDetector
from modules.light_sensor import LightSensor
import hiwonder.ActionGroupControl as AGC

print("[INIT] System Starting...")
voice = WonderEcho()
vision = VisionDetector()
light = LightSensor(pin=24)

def run_task(name, action_group):
    speak(f"Starting {name}")
    # Reset head
    vision.move_head(1500, 1500)
    time.sleep(1)
    
    start_time = time.time()
    while time.time() - start_time < 15: # 15 second timeout
        locked, coords = vision.track_object()
        
        if locked:
            speak("Locked target.")
            # If target is too high/low, we might need to walk forward
            if coords['y'] < 0.6:
                AGC.runActionGroup('go_forward_small')
                time.sleep(0.5)
                continue

            AGC.stopAction()
            time.sleep(0.5)
            AGC.runActionGroup(action_group)
            speak(f"{name} Complete.")
            return

        time.sleep(0.02)
    
    speak(f"Could not find {name}")

speak("Tony Pi Ready.")

try:
    while True:
        # 1. Safety Check
        if light.is_dark():
            AGC.stopAction()
            time.sleep(1)
            continue

        # 2. Voice Command
        cmd = voice.get_command()
        if cmd:
            print(f"[CMD] {cmd}")
            
            if cmd == 'greeting': speak("Hello there.")
            elif cmd == 'wake_up': speak("I am listening.")
            elif cmd == 'stop': 
                AGC.stopAction()
                speak("Stopped.")
            
            # Tasks
            elif cmd == 'pick_up_cardboard': run_task("Pickup", "PickUpDiecut")
            elif cmd == 'transport_cardboard': run_task("Transport", "GrabSheet")
            elif cmd == 'do_diecut_peeling': run_task("Peeling", "PeelingAction")
            elif cmd == 'do_sheet_flipover': run_task("Flip", "TurnSheetOver")

        time.sleep(0.02)

except KeyboardInterrupt:
    AGC.stopAction()