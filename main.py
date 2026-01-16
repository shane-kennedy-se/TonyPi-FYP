#!/usr/bin/python3
"""
TonyPi Robot - Complete Monitoring Integration
Sends ALL telemetry data: servo, sensors, vision, logs, battery, status, camera
"""
import cv2
import time
import sys
import threading
import subprocess
import os
import json
import logging
import socket
import platform
import psutil
from datetime import datetime
from http.server import BaseHTTPRequestHandler, HTTPServer
from socketserver import ThreadingMixIn

# --- HARDWARE IMPORTS ---
import hiwonder.Camera as Camera
from modules import voice_module
from modules import vision_module
from modules import light_sensor

# --- MQTT IMPORTS ---
try:
    import paho.mqtt.client as mqtt
    MQTT_AVAILABLE = True
except ImportError:
    MQTT_AVAILABLE = False
    print("⚠️ paho-mqtt not installed. Telemetry disabled.")

# --- TonyPi SDK IMPORTS (for servo/battery data) ---
HARDWARE_SDK_AVAILABLE = False
board = None
try:
    sys.path.append('/home/pi/TonyPi')
    sys.path.append('/home/pi/TonyPi/HiwonderSDK')
    import hiwonder.ros_robot_controller_sdk as rrc
    board = rrc.Board()
    board.enable_reception(True)
    HARDWARE_SDK_AVAILABLE = True
    print("✅ HiWonder SDK loaded for servo/battery monitoring")
except Exception as e:
    print(f"⚠️ HiWonder SDK not available: {e} - Using simulated data")

# --- MQTT CONFIGURATION ---
MQTT_BROKER = os.getenv("MQTT_BROKER", "192.168.149.100")
MQTT_PORT = int(os.getenv("MQTT_PORT", 1883))
ROBOT_ID = os.getenv("ROBOT_ID", "tonypi_fyp")
CAMERA_STREAM_PORT = int(os.getenv("CAMERA_PORT", 8080))

# MQTT Topics
VISION_TOPIC = f"tonypi/vision/{ROBOT_ID}"
LOGS_TOPIC = f"tonypi/logs/{ROBOT_ID}"
SENSOR_TOPIC = f"tonypi/sensors/{ROBOT_ID}"
SERVO_TOPIC = f"tonypi/servos/{ROBOT_ID}"
STATUS_TOPIC = f"tonypi/status/{ROBOT_ID}"
BATTERY_TOPIC = "tonypi/battery"
LOCATION_TOPIC = "tonypi/location"

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

# Camera stream frame for HTTP server
stream_frame = None
stream_lock = threading.Lock()

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

def send_sensor(sensor_type: str, value, unit: str):
    """Send individual sensor reading to monitoring system."""
    global mqtt_client
    if mqtt_client is None or not MQTT_AVAILABLE:
        return
    
    try:
        data = {
            "robot_id": ROBOT_ID,
            "sensor_type": sensor_type,
            "value": value,
            "unit": unit,
            "timestamp": datetime.now().isoformat()
        }
        mqtt_client.publish(SENSOR_TOPIC, json.dumps(data))
    except Exception as e:
        pass

def send_servo_data():
    """Send servo status data to monitoring system."""
    global mqtt_client, board
    if mqtt_client is None or not MQTT_AVAILABLE:
        return
    
    servo_names = ["Left Hip", "Left Knee", "Right Hip", "Right Knee", "Head Pan", "Head Tilt"]
    servos = {}
    
    try:
        for idx in range(1, 7):  # TonyPi has 6 servos
            if HARDWARE_SDK_AVAILABLE and board:
                try:
                    pos = board.get_bus_servo_pulse(idx)
                    temp = board.get_bus_servo_temp(idx)
                    vin = board.get_bus_servo_vin(idx)
                    
                    angle = ((pos - 500) / 500) * 90 if pos else 0.0
                    temperature = temp if temp else 45.0
                    voltage = (vin / 1000.0) if vin else 5.0
                except:
                    angle, temperature, voltage = 0.0, 45.0, 5.0
            else:
                # Simulated servo data
                import random
                angle = random.uniform(-45, 45)
                temperature = random.uniform(40, 55)
                voltage = random.uniform(4.8, 5.2)
            
            alert = "critical" if temperature > 70 else ("warning" if temperature > 60 else "normal")
            
            servos[f"servo_{idx}"] = {
                "id": idx,
                "name": servo_names[idx - 1] if idx <= len(servo_names) else f"Servo {idx}",
                "position": round(angle, 1),
                "temperature": round(temperature, 1),
                "voltage": round(voltage, 2),
                "torque_enabled": True,
                "alert_level": alert
            }
        
        data = {
            "robot_id": ROBOT_ID,
            "servos": servos,
            "servo_count": len(servos),
            "timestamp": datetime.now().isoformat()
        }
        mqtt_client.publish(SERVO_TOPIC, json.dumps(data))
    except Exception as e:
        print(f"⚠️ Error sending servo data: {e}")

def send_battery_status():
    """Send battery status to monitoring system."""
    global mqtt_client, board
    if mqtt_client is None or not MQTT_AVAILABLE:
        return
    
    try:
        if HARDWARE_SDK_AVAILABLE and board:
            voltage_mv = board.get_battery()
            if voltage_mv:
                voltage_v = voltage_mv / 1000.0
                percentage = max(0, min(100, ((voltage_v - 10.0) / 2.6) * 100))
            else:
                voltage_v, percentage = 12.0, 100.0
        else:
            # Simulated battery
            import random
            percentage = random.uniform(70, 100)
            voltage_v = 10.0 + (percentage / 100.0) * 2.6
        
        data = {
            "robot_id": ROBOT_ID,
            "percentage": round(percentage, 1),
            "voltage": round(voltage_v, 2),
            "charging": False,
            "timestamp": datetime.now().isoformat()
        }
        mqtt_client.publish(BATTERY_TOPIC, json.dumps(data))
    except Exception as e:
        pass

def get_local_ip():
    """Get the local IP address of the robot."""
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except:
        return "192.168.149.1"

def get_cpu_temperature():
    """Get CPU temperature."""
    try:
        with open('/sys/class/thermal/thermal_zone0/temp', 'r') as f:
            return float(f.read()) / 1000.0
    except:
        return 45.0

def send_status_update(status="online"):
    """Send robot status with IP, camera URL, and complete Task Manager metrics."""
    global mqtt_client
    if mqtt_client is None or not MQTT_AVAILABLE:
        return
    
    try:
        ip_address = get_local_ip()
        camera_url = f"http://{ip_address}:{CAMERA_STREAM_PORT}/?action=stream"
        
        # Complete Task Manager system info (matches frontend Monitoring page requirements)
        system_info = {
            "cpu_percent": psutil.cpu_percent(interval=0.1),
            "memory_percent": psutil.virtual_memory().percent,
            "disk_usage": psutil.disk_usage('/').percent,
            "cpu_temperature": get_cpu_temperature(),
            "uptime": time.time() - psutil.boot_time(),  # System uptime in seconds
            "hardware_sdk": HARDWARE_SDK_AVAILABLE,
            "platform": platform.platform()
        }
        
        data = {
            "robot_id": ROBOT_ID,
            "status": status,
            "ip_address": ip_address,
            "camera_url": camera_url,
            "system_info": system_info,
            "timestamp": datetime.now().isoformat()
        }
        mqtt_client.publish(STATUS_TOPIC, json.dumps(data))
    except Exception as e:
        pass

def send_location_update(x=0.0, y=0.0, z=0.0):
    """Send robot location/position update."""
    global mqtt_client
    if mqtt_client is None or not MQTT_AVAILABLE:
        return
    
    try:
        data = {
            "robot_id": ROBOT_ID,
            "x": x,
            "y": y,
            "z": z,
            "timestamp": datetime.now().isoformat()
        }
        mqtt_client.publish(LOCATION_TOPIC, json.dumps(data))
    except Exception as e:
        pass

def send_all_sensors():
    """Send all sensor readings (IMU, CPU temp, ultrasonic, etc.)."""
    global board
    
    try:
        # CPU Temperature
        send_sensor("cpu_temperature", get_cpu_temperature(), "C")
        
        # IMU Data (if available)
        if HARDWARE_SDK_AVAILABLE and board:
            try:
                imu = board.get_imu()
                if imu:
                    send_sensor("accelerometer_x", round(imu[0], 3), "m/s^2")
                    send_sensor("accelerometer_y", round(imu[1], 3), "m/s^2")
                    send_sensor("accelerometer_z", round(imu[2], 3), "m/s^2")
                    send_sensor("gyroscope_x", round(imu[3], 2), "deg/s")
                    send_sensor("gyroscope_y", round(imu[4], 2), "deg/s")
                    send_sensor("gyroscope_z", round(imu[5], 2), "deg/s")
            except:
                pass
            
            # Ultrasonic distance sensor (if available via SDK)
            try:
                from hiwonder.Sonar import Sonar
                sonar = Sonar()
                distance = sonar.getDistance()
                if distance != 99999:
                    send_sensor("ultrasonic_distance", round(distance / 10.0, 1), "cm")
            except:
                # Simulated ultrasonic if not available
                import random
                send_sensor("ultrasonic_distance", round(random.uniform(5.0, 200.0), 1), "cm")
        else:
            # Simulated IMU
            import random
            send_sensor("accelerometer_x", round(random.uniform(-0.5, 0.5), 3), "m/s^2")
            send_sensor("accelerometer_y", round(random.uniform(-0.5, 0.5), 3), "m/s^2")
            send_sensor("accelerometer_z", round(random.uniform(9.5, 10.0), 3), "m/s^2")
            send_sensor("gyroscope_x", round(random.uniform(-10, 10), 2), "deg/s")
            send_sensor("gyroscope_y", round(random.uniform(-10, 10), 2), "deg/s")
            send_sensor("gyroscope_z", round(random.uniform(-10, 10), 2), "deg/s")
            # Simulated ultrasonic
            send_sensor("ultrasonic_distance", round(random.uniform(5.0, 200.0), 1), "cm")
    except Exception as e:
        pass

# ==========================================
# 📷 CAMERA STREAMING SERVER (HTTP MJPEG)
# ==========================================
class MJPGHandler(BaseHTTPRequestHandler):
    """HTTP handler for MJPEG camera streaming."""
    
    def log_message(self, format, *args):
        pass  # Suppress logging
    
    def do_GET(self):
        global stream_frame
        
        if self.path == '/?action=snapshot':
            with stream_lock:
                frame = stream_frame.copy() if stream_frame is not None else None
            
            if frame is not None:
                ret, jpg = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 90])
                if ret:
                    self.send_response(200)
                    self.send_header('Content-type', 'image/jpeg')
                    self.send_header('Access-Control-Allow-Origin', '*')
                    self.end_headers()
                    self.wfile.write(jpg.tobytes())
                    return
            self.send_error(503, 'No frame available')
        else:
            # MJPEG stream
            self.send_response(200)
            self.send_header('Content-type', 'multipart/x-mixed-replace; boundary=--boundarydonotcross')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            
            while running:
                try:
                    with stream_lock:
                        frame = stream_frame.copy() if stream_frame is not None else None
                    
                    if frame is not None:
                        ret, jpg = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 70])
                        if ret:
                            self.wfile.write(b'--boundarydonotcross\r\n')
                            self.send_header('Content-type', 'image/jpeg')
                            self.send_header('Content-length', len(jpg.tobytes()))
                            self.end_headers()
                            self.wfile.write(jpg.tobytes())
                    time.sleep(0.033)  # ~30fps
                except:
                    break

class ThreadedHTTPServer(ThreadingMixIn, HTTPServer):
    daemon_threads = True

def start_camera_server():
    """Start the MJPEG camera streaming server in background."""
    try:
        server = ThreadedHTTPServer(('', CAMERA_STREAM_PORT), MJPGHandler)
        server_thread = threading.Thread(target=server.serve_forever, daemon=True)
        server_thread.start()
        print(f"📷 Camera stream server started on port {CAMERA_STREAM_PORT}")
        print(f"   Stream URL: http://{get_local_ip()}:{CAMERA_STREAM_PORT}/?action=stream")
        return server
    except Exception as e:
        print(f"⚠️ Camera server failed to start: {e}")
        return None

# ==========================================
# 📊 TELEMETRY WORKER THREAD
# ==========================================
def telemetry_worker():
    """Background thread to send periodic telemetry data."""
    global running
    
    last_servo_time = 0
    last_battery_time = 0
    last_sensor_time = 0
    last_status_time = 0
    last_location_time = 0
    
    # Simple location tracking (can be enhanced with actual odometry)
    robot_location = {"x": 0.0, "y": 0.0, "z": 0.0}
    
    print("📊 Telemetry worker started...")
    
    while running:
        try:
            current_time = time.time()
            
            # Send servo data every 3 seconds
            if current_time - last_servo_time >= 3:
                send_servo_data()
                last_servo_time = current_time
            
            # Send battery status every 30 seconds
            if current_time - last_battery_time >= 30:
                send_battery_status()
                last_battery_time = current_time
            
            # Send all sensors every 2 seconds
            if current_time - last_sensor_time >= 2:
                send_all_sensors()
                last_sensor_time = current_time
            
            # Send location update every 5 seconds
            if current_time - last_location_time >= 5:
                send_location_update(robot_location["x"], robot_location["y"], robot_location["z"])
                last_location_time = current_time
            
            # Send status every 10 seconds (includes camera_url and ip_address)
            if current_time - last_status_time >= 10:
                send_status_update("online")
                last_status_time = current_time
            
            time.sleep(0.5)
        except Exception as e:
            time.sleep(1)

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
    global latest_frame, running, latest_result, stream_frame
    print("=" * 50)
    print("   TONYPI ROBOT: COMPLETE MONITORING SYSTEM")
    print("=" * 50)
    print(f"   Robot ID: {ROBOT_ID}")
    print(f"   MQTT Broker: {MQTT_BROKER}:{MQTT_PORT}")
    print(f"   Camera Stream Port: {CAMERA_STREAM_PORT}")
    print("=" * 50)

    # 0. Initialize MQTT Telemetry
    print("\n📡 Connecting to monitoring system...")
    mqtt_connected = init_mqtt()
    if mqtt_connected:
        print("✅ MQTT Telemetry enabled")
    else:
        print("⚠️ MQTT Telemetry disabled - running standalone")

    # 1. Start Camera Streaming Server (HTTP MJPEG)
    print("\n📷 Starting camera stream server...")
    camera_server = start_camera_server()

    # 2. Setup Hardware
    print("\n🔧 Initializing hardware...")
    voice = voice_module.WonderEcho()
    vision = vision_module.VisionController()
    sensor = light_sensor.LightSensor(pin=24)
    print("✅ Voice module initialized")
    print("✅ Vision module initialized")
    print("✅ Light sensor initialized")

    # 3. Open Camera
    print("\n📷 Opening Hiwonder Camera...")
    send_log("INFO", "Opening camera...", "main")
    try:
        cap = Camera.Camera()
        cap.camera_open()
        send_log("INFO", "Camera opened successfully", "main")
        print("✅ Camera opened successfully")
    except Exception as e:
        print(f"❌ CRITICAL: Could not open camera. {e}")
        send_log("ERROR", f"Camera open failed: {e}", "main")
        return

    # 4. Start Vision AI Thread
    ai_thread = threading.Thread(target=inference_worker, args=(vision,), daemon=True)
    ai_thread.start()
    send_log("INFO", "AI Vision thread started", "vision")
    print("✅ AI Vision thread started")

    # 5. Start Telemetry Worker Thread (sends servo, battery, sensors)
    telemetry_thread = threading.Thread(target=telemetry_worker, daemon=True)
    telemetry_thread.start()
    print("✅ Telemetry worker started")

    # 6. Send initial status update
    send_status_update("online")
    send_battery_status()
    
    current_state = STATE_IDLE
    current_task = None
    was_dark_last_frame = False
    last_telemetry_time = 0
    
    # Initial Voice Check
    voice_module.speak("System online.")
    print("\n" + "=" * 50)
    print("✅ SYSTEM READY - All monitoring active!")
    print("=" * 50)
    local_ip = get_local_ip()
    camera_url = f"http://{local_ip}:{CAMERA_STREAM_PORT}/?action=stream"
    print(f"📷 Camera Stream: {camera_url}")
    print(f"📡 Robot IP: {local_ip}")
    print("Press 'q' in the camera window to quit")
    print("=" * 50 + "\n")
    send_log("INFO", "System ready and operational", "main")
    
    # Send status again after everything is ready (ensures camera_url is sent)
    time.sleep(1)  # Small delay to ensure MQTT is fully connected
    send_status_update("online")
    print(f"📡 Sent camera URL to monitoring system: {camera_url}")

    try:
        while True:
            # A. UPDATE CAMERA
            ret, frame = cap.read()
            if not ret: 
                time.sleep(0.01)
                continue
            
            # Update frame for vision processing
            with frame_lock: 
                latest_frame = frame
            
            # Update frame for HTTP streaming (with annotations later)
            # Will be updated again after drawing annotations

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
                
                # Update stream frame with warning overlay
                with stream_lock:
                    stream_frame = frame.copy()
                
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

            # Update stream frame for HTTP server (with all annotations)
            with stream_lock:
                stream_frame = frame.copy()
            
            cv2.imshow("TonyPi", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'): break

    except KeyboardInterrupt:
        print("\n🛑 Stopping...")
        send_log("INFO", "Shutdown requested by user", "main")
    finally:
        print("🔄 Cleaning up...")
        running = False
        
        # Send offline status before disconnecting
        send_status_update("offline")
        send_log("INFO", "Robot shutting down", "main")
        time.sleep(0.5)  # Allow final messages to send
        
        # Cleanup hardware
        sensor.cleanup()
        cap.camera_close()
        cv2.destroyAllWindows()
        
        # Wait for threads
        ai_thread.join(timeout=2)
        
        # Cleanup MQTT
        global mqtt_client
        if mqtt_client:
            mqtt_client.loop_stop()
            mqtt_client.disconnect()
            print("📡 Disconnected from monitoring system")
        
        print("✅ Shutdown complete")

if __name__ == "__main__":
    main()