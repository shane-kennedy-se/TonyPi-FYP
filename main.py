#!/usr/bin/python3
# coding=utf8
import sys
import time
import os
import shutil
import cv2 
import numpy as np 
from dotenv import load_dotenv

# --- PATH SETUP ---
sys.path.append('/home/pi/TonyPi/HiwonderSDK')
sys.path.append('/home/pi/TonyPi')

# --- IMPORTS ---
from modules.light_sensor import LightSensor
from modules.voice_module import WonderEcho, speak
from modules.vision_detector import VisionDetector
from modules.ai_brain import get_dynamic_response, generate_smart_greeting, analyze_obstacle
import hiwonder.ros_robot_controller_sdk as rrc
from hiwonder.Controller import Controller
import hiwonder.ActionGroupControl as AGC
import hiwonder.yaml_handle as yaml_handle

# --- INIT DRIVERS ---
print("[INIT] Loading Robot Drivers...")
board = rrc.Board()
ctl = Controller(board)

# --- INIT MODULES ---
light = LightSensor(pin=24) 
vision = VisionDetector() # Camera is handled inside here

# Voice Init (Safe Mode)
voice = None
try:
    voice = WonderEcho()
    print("[INIT] Voice Module Ready (Hex Mode).")
except Exception as e:
    print(f"[INIT] Voice Module Failed: {e}")

# --- CONFIGURATION ---
SYSTEM_ACTION_PATH = "/home/pi/TonyPi/ActionGroups/"
MY_ACTION_PATH = "actions/"

def setup_actions():
    """Ensures action files are in the right place"""
    if not os.path.exists(MY_ACTION_PATH): return
    for f in os.listdir(MY_ACTION_PATH):
        if f.endswith('.d6a'):
            try:
                shutil.copy(os.path.join(MY_ACTION_PATH, f), os.path.join(SYSTEM_ACTION_PATH, f))
            except: pass
setup_actions()

# --- HELPER: SMART SPEAK ---
def smart_speak(text):
    """Speaks safely and clears the microphone buffer"""
    if not text: return
    print(f"[ROBOT] {text}")
    
    # Speak using Piper TTS (from voice_module)
    speak(text)

    # Wait for audio to finish (approx calculation)
    # 1.0s base + 0.08s per character
    time.sleep(1.0 + len(text) * 0.08)
    
    # Clear microphone buffer so we don't hear ourselves
    if voice and voice.ser:
        try:
            voice.ser.flushInput()
        except:
            pass

# --- HELPER: OBSTACLE CHECK ---
def check_for_obstacles():
    """Checks for obstacles using Vision (if available) or Sensors"""
    # Note: If you have the VL53L0X sensor, we can add that specific code here.
    # For now, we use the camera to check if the view is blocked (dark/close)
    pass 

# --- TASK FUNCTION ---
def perform_visual_task(task_name, action_name):
    smart_speak(f"Scanning for {task_name}")
    start_time = time.time()
    
    # Reset Head to Center
    if hasattr(vision, 'move_head'):
        vision.move_head(1500, 1500)
    time.sleep(0.5)

    while True:
        # Safety Check
        if light.is_dark():
            smart_speak("Too dark. Pausing.")
            return

        # TRACKING
        is_locked, coords = vision.track_object()
        
        if is_locked:
            x, y = coords['x'], coords['y']
            
            # --- WALK ALIGNMENT LOGIC ---
            # X is Left/Right (0.0 to 1.0)
            if x < 0.45:
                # print(f"[ALIGN] Left -> Turn Left")
                AGC.runActionGroup('turn_left_small')
                time.sleep(0.5)
                continue
            elif x > 0.55:
                # print(f"[ALIGN] Right -> Turn Right")
                AGC.runActionGroup('turn_right_small')
                time.sleep(0.5)
                continue
                
            # Y is Distance (0.0 to 1.0)
            if y < 0.55: # Far away
                # print(f"[ALIGN] Far -> Forward")
                AGC.runActionGroup('go_forward_small')
                time.sleep(0.5)
                continue
            elif y > 0.65: # Too close
                # print(f"[ALIGN] Close -> Back")
                AGC.runActionGroup('back_small')
                time.sleep(0.5)
                continue
            
            # --- EXECUTION ---
            smart_speak("Target locked.")
            AGC.stopAction()
            time.sleep(0.5)
            
            # Center head before arm movement
            if hasattr(vision, 'move_head'):
                vision.move_head(1500, 1500)
            time.sleep(0.5)
            
            smart_speak("Executing.")
            AGC.runActionGroup(action_name)
            smart_speak("Done.")
            return

        # Timeout
        if time.time() - start_time > 15:
            smart_speak("Target not found.")
            return
        time.sleep(0.01)

# --- MAIN LOOP ---
smart_speak("Tony Pi Online.")

try:
    while True:
        # 1. LIGHT SENSOR SAFETY
        if light.is_dark():
            AGC.stopAction()
            smart_speak("Darkness detected. Stopping.")
            time.sleep(2)
            continue

        # 2. VOICE COMMANDS
        cmd = None
        if voice:
            try:
                cmd = voice.get_command()
            except:
                pass 

        if cmd:
            print(f"[CMD] {cmd}")
            
            if cmd == 'greeting':
                smart_speak("Scanning.")
                ret, frame = vision.get_frame()
                if ret:
                    reply = generate_smart_greeting(frame)
                    smart_speak(reply)
                else:
                    smart_speak("Camera error.")
            
            elif cmd == 'pick_up_cardboard':
                perform_visual_task("cardboard", "PickUpDiecut")
            
            elif cmd == 'flip_over':
                perform_visual_task("flip target", "TurnSheetOver") 
                
            elif cmd == 'transport_cardboard':
                perform_visual_task("transport target", "GrabSheet")
                
            elif cmd == 'peeling':
                perform_visual_task("peeling target", "GrabSheet")
                
            elif cmd == 'label_insertion':
                perform_visual_task("label target", "PickUpDiecut")
            
            elif cmd == 'stop':
                AGC.stopAction()
                smart_speak("Stopping.")
                time.sleep(2) 

            elif cmd == 'sleep':
                smart_speak("Entering sleep mode.")
                # We can add a sleep loop here if needed
                
            elif cmd == 'wake_up':
                smart_speak("I am awake.")
            
            else:
                # 3. AI CHAT (Gemini Fallback)
                # If the module sent raw text (not a command ID), use Gemini
                reply = get_dynamic_response(cmd)
                smart_speak(reply)

        time.sleep(0.05)

except KeyboardInterrupt:
    if voice: voice.close()
    light.cleanup()
    print("[EXIT] System Shutdown.")