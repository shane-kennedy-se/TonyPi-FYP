#!/usr/bin/python3
import cv2
import time
import sys
import threading
import subprocess
import os
import json
import logging
from datetime import datetime

# --- HARDWARE IMPORTS ---
import hiwonder.Camera as Camera
from modules import voice_module
from modules import vision_module
from modules import light_sensor  # <--- NEW IMPORT

# --- MQTT IMPORTS ---
try:
    import paho.mqtt.client as mqtt
    MQTT_AVAILABLE = True
except ImportError:
    MQTT_AVAILABLE = False
    print("⚠️ paho-mqtt not installed. Telemetry disabled.")

# --- MQTT CONFIGURATION ---
MQTT_BROKER = os.getenv("MQTT_BROKER", "192.168.149.100")  # Default to monitoring server
MQTT_PORT = int(os.getenv("MQTT_PORT", 1883))
ROBOT_ID = os.getenv("ROBOT_ID", "tonypi_fyp")

# MQTT Topics
VISION_TOPIC = f"tonypi/vision/{ROBOT_ID}"
LOGS_TOPIC = f"tonypi/logs/{ROBOT_ID}"
SENSOR_TOPIC = f"tonypi/sensors/{ROBOT_ID}"

# Global MQTT client
mqtt_client = None

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
# 📡 MQTT TELEMETRY FUNCTIONS
# ==========================================
def init_mqtt():
    """Initialize MQTT connection for telemetry."""
    global mqtt_client
    if not MQTT_AVAILABLE:
        return False
    
    try:
        mqtt_client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=ROBOT_ID)
        mqtt_client.on_connect = on_mqtt_connect
        mqtt_client.on_disconnect = on_mqtt_disconnect
        mqtt_client.connect(MQTT_BROKER, MQTT_PORT, 60)
        mqtt_client.loop_start()
        return True
    except Exception as e:
        print(f"⚠️ MQTT connection failed: {e}")
        return False

def on_mqtt_connect(client, userdata, flags, rc, properties=None):
    """Callback when MQTT connects."""
    if rc == 0:
        print(f"📡 Connected to MQTT broker at {MQTT_BROKER}:{MQTT_PORT}")
        send_log("INFO", "Robot connected to monitoring system", "main")
    else:
        print(f"⚠️ MQTT connection failed with code: {rc}")

def on_mqtt_disconnect(client, userdata, flags=None, rc=None, properties=None):
    """Callback when MQTT disconnects."""
    print("📡 Disconnected from MQTT broker")

def send_vision_data(detection, state, nav_cmd=None, error=None, is_locked=False):
    """Send vision detection data to monitoring system."""
    global mqtt_client
    if mqtt_client is None or not MQTT_AVAILABLE:
        return
    
    try:
        data = {
            "robot_id": ROBOT_ID,
            "timestamp": datetime.now().isoformat(),
            "state": state,
            "is_locked": is_locked
        }
        
        if detection:
            label, conf, box, cx = detection
            data.update({
                "detection": True,
                "label": label,
                "confidence": round(conf, 3),
                "bbox": {"x1": box[0], "y1": box[1], "x2": box[2], "y2": box[3]},
                "center_x": cx,
                "frame_width": FRAME_WIDTH
            })
        else:
            data["detection"] = False
        
        if nav_cmd:
            data["navigation_command"] = nav_cmd
        if error is not None:
            data["error"] = error
        
        mqtt_client.publish(VISION_TOPIC, json.dumps(data))
    except Exception as e:
        print(f"⚠️ Error sending vision data: {e}")

def send_log(level, message, source="main"):
    """Send log message to monitoring system."""
    global mqtt_client
    if mqtt_client is None or not MQTT_AVAILABLE:
        return
    
    try:
        data = {
            "robot_id": ROBOT_ID,
            "timestamp": datetime.now().isoformat(),
            "level": level,
            "message": message,
            "source": source
        }
        mqtt_client.publish(LOGS_TOPIC, json.dumps(data))
    except Exception as e:
        pass  # Silent fail for logs

def send_light_sensor_data(is_dark, pin=24):
    """Send light sensor data to monitoring system."""
    global mqtt_client
    if mqtt_client is None or not MQTT_AVAILABLE:
        return
    
    try:
        data = {
            "robot_id": ROBOT_ID,
            "sensor_type": "light_sensor",
            "value": 1 if is_dark else 0,
            "timestamp": datetime.now().isoformat(),
            "unit": "bool",
            "is_dark": is_dark,
            "pin": pin
        }
        mqtt_client.publish(SENSOR_TOPIC, json.dumps(data))
    except Exception as e:
        pass

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

    # 0. Initialize MQTT Telemetry
    print("📡 Connecting to monitoring system...")
    mqtt_connected = init_mqtt()
    if mqtt_connected:
        print("✅ Telemetry enabled")
    else:
        print("⚠️ Telemetry disabled - running standalone")

    # 1. Setup Hardware
    voice = voice_module.WonderEcho()
    vision = vision_module.VisionController()
    
    # Initialize your specific Light Sensor on Pin 24
    sensor = light_sensor.LightSensor(pin=24)

    # 2. Open Camera
    print("📷 Opening Hiwonder Camera...")
    send_log("INFO", "Opening camera...", "main")
    try:
        cap = Camera.Camera()
        cap.camera_open()
        send_log("INFO", "Camera opened successfully", "main")
    except Exception as e:
        print(f"❌ CRITICAL: Could not open camera. {e}")
        send_log("ERROR", f"Camera open failed: {e}", "main")
        return

    # 3. Start Vision Brain
    ai_thread = threading.Thread(target=inference_worker, args=(vision,))
    ai_thread.daemon = True
    ai_thread.start()
    send_log("INFO", "AI Vision thread started", "vision")

    current_state = STATE_IDLE
    current_task = None
    was_dark_last_frame = False
    last_telemetry_time = 0
    
    # Initial Voice Check
    voice_module.speak("System online.")
    print("✅ System Ready.")
    send_log("INFO", "System ready and operational", "main")

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
            
            # Send light sensor telemetry periodically
            current_time = time.time()
            if current_time - last_telemetry_time >= 2.0:
                send_light_sensor_data(is_dark_now, pin=24)
                last_telemetry_time = current_time
            
            if is_dark_now:
                # 🛑 DANGER: TOO DARK!
                if not was_dark_last_frame:
                    voice_module.speak("Too dark. Stopping now.")
                    send_log("WARNING", "Darkness detected - robot stopped", "safety")
                
                # If we were doing something, STOP and SPEAK.
                if current_state != STATE_IDLE:
                    print("⚠️ DARKNESS DETECTED! ABORTING ACTION!")
                    send_log("WARNING", f"Aborting action due to darkness. Previous state: {current_state}", "safety")
                    current_state = STATE_IDLE
                    current_task = None
                    vision.reset() # Reset vision memory
                
                # Visual Warning on Screen
                cv2.rectangle(frame, (0,0), (FRAME_WIDTH, FRAME_HEIGHT), (0,0,255), 5)
                cv2.putText(frame, "⚠️ TOO DARK - STOPPED", (150, 240), 
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 3)
                
                # Send vision data showing stopped state
                send_vision_data(None, current_state, nav_cmd="STOPPED_DARK")
                
                was_dark_last_frame = True
                
                # Skip the rest of the loop (Don't listen or look)
                cv2.imshow("TonyPi", frame)
                if cv2.waitKey(1) & 0xFF == ord('q'): break
                continue
            
            # If light just came back on, notify user ONE time
            if was_dark_last_frame and not is_dark_now:
                voice_module.speak("Light levels normal. System ready.")
                send_log("INFO", "Light levels restored - system ready", "safety")
                was_dark_last_frame = False

            # ==========================================
            # 🎤 VOICE COMMANDS
            # ==========================================
            if current_state != STATE_ACTING:
                cmd = voice.get_command()
                if cmd:
                    print(f"🎤 Command: {cmd}")
                    send_log("INFO", f"Voice command received: {cmd}", "voice")
                    
                    if cmd == "Wake Up":
                        voice_module.speak("Listening.")
                        current_state = STATE_IDLE
                        send_log("INFO", "Wake up command - listening", "voice")
                    
                    elif cmd == "Stop":
                        voice_module.speak("Stopping.")
                        current_state = STATE_IDLE
                        vision.reset()
                        send_log("INFO", "Stop command - robot stopped", "voice")
                    
                    elif cmd in ["Peeling", "Insert Label", "Transport", "Flip"]:
                        # Final check before starting
                        if not sensor.is_dark():
                            current_task = cmd
                            voice_module.speak(f"Starting {cmd}. Searching.")
                            current_state = STATE_SEARCHING
                            vision.reset()  # Clear old vision memory
                            send_log("INFO", f"Starting task: {cmd}", "voice")
                        else:
                            voice_module.speak("Cannot start. It is too dark.")
                            send_log("WARNING", f"Task {cmd} rejected - too dark", "voice")

            # ==========================================
            # 🤖 ROBOT LOGIC
            # ==========================================
            if current_state == STATE_IDLE:
                cv2.putText(frame, "IDLE", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                # Send idle state vision data periodically
                if current_time - last_telemetry_time >= 2.0:
                    send_vision_data(None, current_state)

            elif current_state == STATE_SEARCHING:
                current_det = None
                with result_lock: current_det = latest_result
                
                if current_det:
                    label, conf, box, cx = current_det
                    x1, y1, x2, y2 = box
                    nav_cmd, error = vision.get_navigation_command(cx, FRAME_WIDTH)
                    
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    
                    # Send vision detection data
                    is_locked = (nav_cmd == "LOCKED")
                    send_vision_data(current_det, current_state, nav_cmd=nav_cmd, error=error, is_locked=is_locked)

                    if nav_cmd == "LOCKED":
                        cv2.putText(frame, "LOCKED", (x1, y1-30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255), 2)
                        voice_module.speak("Target locked.")
                        send_log("INFO", f"Target locked: {label} (conf: {conf:.2f})", "vision")
                        current_state = STATE_ACTING
                        
                    elif nav_cmd == "TURN_LEFT":
                        cv2.arrowedLine(frame, (320, 240), (270, 240), (255, 255, 0), 3)
                    elif nav_cmd == "TURN_RIGHT":
                        cv2.arrowedLine(frame, (320, 240), (370, 240), (255, 255, 0), 3)
                else:
                    cv2.putText(frame, "Scanning...", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 165, 255), 2)
                    # Send scanning state (no detection)
                    send_vision_data(None, current_state, nav_cmd="SCANNING")

            elif current_state == STATE_ACTING:
                cv2.putText(frame, f"TASK: {current_task}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                
                print(f"[Main] Running action: {current_task}")
                send_log("INFO", f"Executing action: {current_task}", "action")
                success = vision.run_action(current_task)
                
                if not success:
                    send_log("WARNING", f"Action {current_task} may have failed", "action")
                    time.sleep(3) 
                
                voice_module.speak(f"{current_task} complete.")
                send_log("INFO", f"Action completed: {current_task}", "action")
                current_state = STATE_IDLE
                current_task = None

            cv2.imshow("TonyPi", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'): break

    except KeyboardInterrupt:
        print("Stopping...")
        send_log("INFO", "Shutdown requested by user", "main")
    finally:
        running = False
        send_log("INFO", "Robot shutting down", "main")
        sensor.cleanup() # Clean up Pin 24
        cap.camera_close()
        cv2.destroyAllWindows()
        ai_thread.join()
        
        # Cleanup MQTT
        global mqtt_client
        if mqtt_client:
            mqtt_client.loop_stop()
            mqtt_client.disconnect()
            print("📡 Disconnected from monitoring system")

if __name__ == "__main__":
    main()