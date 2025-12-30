#!/usr/bin/python3
import sys
import time
import os

# --- AUTO-FIX AUDIO PERMISSIONS ---
os.environ["XDG_RUNTIME_DIR"] = "/run/user/1000"

# --- PATH SETUP (For TonyPi SDK) ---
# Ensure these match where the factory SDK is actually installed
sys.path.append('/home/pi/TonyPi/HiwonderSDK')
sys.path.append('/home/pi/TonyPi')

from modules.light_sensor import LightSensor
from modules.voice_module import WonderEcho, speak
from modules.vision_detector import VisionDetector
import hiwonder.ActionGroupControl as AGC

print("[INIT] Starting TonyPi System (Ryan Voice)...")
light = LightSensor(pin=24)
vision = VisionDetector()

voice = None
try:
    voice = WonderEcho()
    print("[INIT] Voice Module Active.")
except:
    print("[INIT] Voice Module Failed.")

def run_visual_task(task_name, action_file):
    speak(f"Starting {task_name}")
    start_t = time.time()
    vision.move_head(1500, 1500)
    time.sleep(0.5)
    
    while time.time() - start_t < 15:
        if light.is_dark():
            speak("Too dark.")
            return
        
        locked, coords = vision.track_object()
        if locked:
            if coords['y'] < 0.55:
                AGC.runActionGroup('go_forward_small')
                continue
            
            speak("Locked.")
            AGC.stopAction()
            time.sleep(0.5)
            AGC.runActionGroup(action_file)
            speak("Done.")
            return
        time.sleep(0.01)
    speak("Target not found.")

speak("Tony Pi Online.")

try:
    while True:
        if light.is_dark():
            AGC.stopAction()
            time.sleep(1)
            continue

        if voice:
            cmd = voice.get_command()
            if cmd:
                print(f"[CMD] {cmd}")
                
                if cmd == 'greeting': speak("Welcome.")
                elif cmd == 'hello_tony': speak("Yes boss?")
                elif cmd == 'sleep': speak("Taking a break.")
                elif cmd == 'forward': AGC.runActionGroup('go_forward')
                elif cmd == 'back': AGC.runActionGroup('back')
                elif cmd == 'turn_left': AGC.runActionGroup('turn_left')
                elif cmd == 'turn_right': AGC.runActionGroup('turn_right')
                elif cmd == 'stop':
                    AGC.stopAction()
                    speak("Stopped.")
                elif cmd == 'pick_up_cardboard': run_visual_task("pickup", "PickUpDiecut")
                elif cmd == 'transport_cardboard': run_visual_task("transport", "GrabSheet")
                elif cmd == 'do_diecut_peeling': run_visual_task("peeling", "PeelingAction")
                elif cmd == 'do_sheet_flipover': run_visual_task("flip", "TurnSheetOver")
                elif cmd == 'do_label_insertion': run_visual_task("insert", "InsertLabel")

        time.sleep(0.02)

except KeyboardInterrupt:
    AGC.stopAction()
    if voice: voice.close()