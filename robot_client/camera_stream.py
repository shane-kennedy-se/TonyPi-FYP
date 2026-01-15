#!/usr/bin/env python3
"""
MJPEG Camera Streaming Server for TonyPi Robot
Captures frames from USB camera and serves them as MJPEG stream over HTTP.
Integrated with light sensor for low-light warning popup.

Usage:
    python camera_stream.py [--port 8080]
    
Access the stream at:
    http://<tonypi-ip>:8080/?action=stream
    http://<tonypi-ip>:8080/?action=snapshot
"""

import time
import threading
import argparse
import random
from http.server import BaseHTTPRequestHandler, HTTPServer
from socketserver import ThreadingMixIn

# Try to import OpenCV
try:
    import cv2
    import numpy as np
    CV2_AVAILABLE = True
except ImportError:
    CV2_AVAILABLE = False
    print("Warning: OpenCV not available. Camera will run in simulation mode.")

# Try to import GPIO for light sensor
LIGHT_SENSOR_AVAILABLE = False
try:
    import RPi.GPIO as GPIO
    LIGHT_SENSOR_AVAILABLE = True
    print("RPi.GPIO available for light sensor")
except ImportError:
    print("Warning: RPi.GPIO not available - light sensor will be simulated")

# ==========================================
# LIGHT SENSOR CLASS (same as FYP_Robot)
# ==========================================
class LightSensor:
    """
    Light sensor class for reading ambient light via GPIO.
    Falls back to simulation mode if GPIO is not available.
    """
    
    def __init__(self, pin=24):
        self.pin = pin
        self.initialized = False
        self._last_dark_state = False
        self._state_change_time = 0
        
        if LIGHT_SENSOR_AVAILABLE:
            try:
                GPIO.setwarnings(False)
                GPIO.setmode(GPIO.BCM)
                GPIO.setup(self.pin, GPIO.IN)
                self.initialized = True
                print(f"Light sensor initialized on GPIO pin {pin}")
            except Exception as e:
                print(f"Failed to initialize light sensor: {e}")
    
    def is_dark(self) -> bool:
        """
        Returns True if sensor detects darkness (blocked/low light).
        Same logic as FYP_Robot light_sensor.py
        """
        if self.initialized and LIGHT_SENSOR_AVAILABLE:
            try:
                # Returns True if Sensor is blocked (High signal)
                return GPIO.input(self.pin) == 1
            except Exception as e:
                print(f"Error reading light sensor: {e}")
                return False
        
        # Simulation mode - randomly simulate light conditions
        # 5% chance of being dark (for testing purposes)
        return random.random() < 0.05
    
    def cleanup(self):
        """Clean up GPIO resources."""
        if self.initialized and LIGHT_SENSOR_AVAILABLE:
            try:
                GPIO.cleanup(self.pin)
            except Exception as e:
                print(f"Error cleaning up light sensor: {e}")


# Global frame storage
img_show = None
quality = (int(cv2.IMWRITE_JPEG_QUALITY) if CV2_AVAILABLE else 0, 70)

# Light sensor instance (global for access in camera task)
light_sensor = None
is_dark_warning = False  # Global flag to track dark state for overlay


class Camera:
    """
    Camera capture class using OpenCV.
    Runs frame capture in a background thread.
    """
    
    def __init__(self, resolution=(640, 480), device=-1):
        """
        Initialize camera.
        
        Args:
            resolution: Tuple of (width, height)
            device: Camera device index (-1 for auto-detect)
        """
        self.cap = None
        self.width = resolution[0]
        self.height = resolution[1]
        self.device = device
        self.frame = None
        self.opened = False
        self.flip = False
        self.flip_param = 0
        self.simulation_mode = not CV2_AVAILABLE
        
        # Try to load camera settings
        try:
            import yaml
            with open('/boot/camera_setting.yaml', 'r') as f:
                camera_setting = yaml.safe_load(f)
                self.flip = camera_setting.get('flip', False)
                self.flip_param = camera_setting.get('flip_param', 0)
        except:
            pass
        
        # Start capture thread
        self.running = True
        self.th = threading.Thread(target=self.camera_task, daemon=True)
        self.th.start()

    def camera_open(self):
        """Open the camera."""
        if self.simulation_mode:
            self.opened = True
            return True
            
        try:
            self.cap = cv2.VideoCapture(self.device)
            if self.cap.isOpened():
                self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('Y', 'U', 'Y', 'V'))
                self.cap.set(cv2.CAP_PROP_FPS, 30)
                self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
                self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
                self.opened = True
                print(f"Camera opened: {self.width}x{self.height}")
                return True
            else:
                print("Failed to open camera")
                return False
        except Exception as e:
            print(f'Failed to open camera: {e}')
            return False

    def camera_close(self):
        """Close the camera."""
        try:
            self.opened = False
            time.sleep(0.2)
            if self.cap is not None:
                self.cap.release()
                time.sleep(0.05)
            self.cap = None
        except Exception as e:
            print(f'Failed to close camera: {e}')
    
    def read(self):
        """
        Read the current frame.
        
        Returns:
            tuple: (success, frame) where frame is numpy array or None
        """
        if self.frame is None:
            return False, self.frame
        else:
            return True, self.frame
    
    def camera_task(self):
        """Background task for capturing frames with light sensor integration."""
        global light_sensor, is_dark_warning
        
        was_dark_last_frame = False
        
        while self.running:
            try:
                if self.simulation_mode:
                    # Generate test pattern
                    if self.opened:
                        frame = np.zeros((self.height, self.width, 3), dtype=np.uint8)
                        # Add text
                        cv2.putText(frame, "TonyPi Camera", (50, self.height // 2 - 20),
                                    cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
                        cv2.putText(frame, time.strftime("%H:%M:%S"), (50, self.height // 2 + 30),
                                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                        # Add moving element
                        x = int((time.time() % 5) / 5 * self.width)
                        cv2.circle(frame, (x, 50), 20, (0, 0, 255), -1)
                        
                        # Check light sensor and add overlay if dark
                        frame = self._add_light_sensor_overlay(frame)
                        self.frame = frame
                    time.sleep(0.033)  # ~30fps
                    continue
                    
                if self.opened and self.cap is not None and self.cap.isOpened():
                    ret, frame_tmp = self.cap.read()
                    if ret:
                        frame = cv2.resize(frame_tmp, (self.width, self.height), 
                                          interpolation=cv2.INTER_NEAREST)
                        if self.flip:
                            frame = cv2.flip(frame, self.flip_param)
                        
                        # Check light sensor and add overlay if dark
                        frame = self._add_light_sensor_overlay(frame)
                        self.frame = frame
                    else:
                        # Try to reopen camera
                        self.frame = None
                        cap = cv2.VideoCapture(self.device)
                        ret, _ = cap.read()
                        if ret:
                            self.cap = cap
                elif self.opened:
                    cap = cv2.VideoCapture(self.device)
                    ret, _ = cap.read()
                    if ret:
                        self.cap = cap              
                else:
                    time.sleep(0.01)
            except Exception as e:
                print(f'Camera capture error: {e}')
                time.sleep(0.01)
    
    def _add_light_sensor_overlay(self, frame):
        """
        Check light sensor and add visual warning overlay if dark.
        Same visual style as FYP_Robot main.py
        """
        global light_sensor, is_dark_warning
        
        if light_sensor is None:
            return frame
        
        try:
            is_dark_now = light_sensor.is_dark()
            is_dark_warning = is_dark_now  # Update global state
            
            if is_dark_now:
                # ==========================================
                # LOW LIGHT WARNING POPUP (same as FYP_Robot)
                # ==========================================
                # Draw thick red border around the entire frame
                cv2.rectangle(frame, (0, 0), (self.width - 1, self.height - 1), (0, 0, 255), 8)
                
                # Create semi-transparent overlay at top
                overlay = frame.copy()
                cv2.rectangle(overlay, (0, 0), (self.width, 80), (0, 0, 100), -1)
                cv2.addWeighted(overlay, 0.6, frame, 0.4, 0, frame)
                
                # Warning text - "LOW LIGHT LEVEL DETECTED"
                warning_text = "LOW LIGHT LEVEL DETECTED"
                text_size = cv2.getTextSize(warning_text, cv2.FONT_HERSHEY_SIMPLEX, 0.9, 2)[0]
                text_x = (self.width - text_size[0]) // 2
                cv2.putText(frame, warning_text, (text_x, 35), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)
                
                # Sub-warning text
                sub_text = "Vision system may not work properly"
                sub_size = cv2.getTextSize(sub_text, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 1)[0]
                sub_x = (self.width - sub_size[0]) // 2
                cv2.putText(frame, sub_text, (sub_x, 60), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
                
                # Add timestamp
                cv2.putText(frame, time.strftime("%H:%M:%S"), (10, self.height - 10),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
                
                # Print warning to console (only once when state changes)
                if not hasattr(self, '_was_dark') or not self._was_dark:
                    print("WARNING: Low light level detected! Camera vision may be affected.")
                    self._was_dark = True
            else:
                # Light is normal - reset state
                if hasattr(self, '_was_dark') and self._was_dark:
                    print("INFO: Light levels restored to normal.")
                self._was_dark = False
        
        except Exception as e:
            print(f"Error checking light sensor: {e}")
        
        return frame


class MJPGHandler(BaseHTTPRequestHandler):
    """HTTP request handler for MJPEG streaming."""
    
    def log_message(self, format, *args):
        """Suppress default logging."""
        pass
    
    def do_GET(self):
        """Handle GET requests."""
        global img_show
        
        if self.path == '/?action=snapshot':
            # Return single JPEG snapshot
            if img_show is not None:
                try:
                    ret, jpg = cv2.imencode('.jpg', img_show, 
                                           [int(cv2.IMWRITE_JPEG_QUALITY), 100])
                    jpg_bytes = jpg.tobytes()
                    self.send_response(200)
                    self.send_header('Content-type', 'image/jpeg')
                    self.send_header('Content-length', len(jpg_bytes))
                    self.send_header('Access-Control-Allow-Origin', '*')
                    self.end_headers()
                    self.wfile.write(jpg_bytes)
                except Exception as e:
                    print(f'Snapshot error: {e}')
                    self.send_error(500, str(e))
            else:
                self.send_error(503, 'No frame available')
        else:
            # MJPEG stream
            self.send_response(200)
            self.send_header('Content-type', 'multipart/x-mixed-replace; boundary=--boundarydonotcross')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            
            while True:
                try:
                    if img_show is not None:
                        ret, jpg = cv2.imencode('.jpg', img_show, 
                                               [int(cv2.IMWRITE_JPEG_QUALITY), 70,
                                                cv2.IMWRITE_JPEG_OPTIMIZE, 1])
                        jpg_bytes = jpg.tobytes()
                        
                        self.wfile.write(b'--boundarydonotcross\r\n')
                        self.send_header('Content-type', 'image/jpeg')
                        self.send_header('Content-length', len(jpg_bytes))
                        self.end_headers()
                        self.wfile.write(jpg_bytes)
                    else:
                        time.sleep(0.1)
                except Exception as e:
                    print(f"Stream error: {e}")
                    break


class ThreadedHTTPServer(ThreadingMixIn, HTTPServer):
    """Handle requests in separate threads."""
    daemon_threads = True
    allow_reuse_address = True  # Allow reuse of address after server closes


def start_camera_server(port=8080, camera_device=-1, resolution=(640, 480), light_sensor_pin=24):
    """
    Start the MJPEG camera streaming server with light sensor integration.
    
    Args:
        port: HTTP server port (default: 8080)
        camera_device: Camera device index (-1 for auto)
        resolution: Camera resolution tuple (width, height)
        light_sensor_pin: GPIO pin for light sensor (default: 24)
    """
    global img_show, light_sensor
    
    print("=" * 50)
    print("   TONYPI CAMERA STREAM SERVER")
    print("=" * 50)
    print(f"   Port: {port}")
    print(f"   Resolution: {resolution[0]}x{resolution[1]}")
    print(f"   OpenCV available: {CV2_AVAILABLE}")
    print(f"   Light sensor GPIO: {light_sensor_pin}")
    print("=" * 50)
    
    # Initialize light sensor (same as FYP_Robot)
    print("\nInitializing light sensor...")
    light_sensor = LightSensor(pin=light_sensor_pin)
    if light_sensor.initialized:
        print(f"Light sensor initialized on GPIO pin {light_sensor_pin}")
    else:
        print("Light sensor running in simulation mode")
    
    # Initialize camera
    print("\nInitializing camera...")
    camera = Camera(resolution=resolution, device=camera_device)
    if not camera.camera_open():
        print("Warning: Camera failed to open, running with test pattern")
    
    # Start HTTP server
    try:
        server = ThreadedHTTPServer(('', port), MJPGHandler)
    except OSError as e:
        if e.errno == 98:  # Address already in use
            print(f"\nError: Port {port} is already in use!")
            print(f"To fix this, run one of these commands:")
            print(f"  sudo fuser -k {port}/tcp   # Kill process using port {port}")
            print(f"  python camera_stream.py --port {port + 1}  # Use different port")
            return
        raise
    server_thread = threading.Thread(target=server.serve_forever, daemon=True)
    server_thread.start()
    
    print(f"Camera server started!")
    print(f"Stream URL: http://localhost:{port}/?action=stream")
    print(f"Snapshot URL: http://localhost:{port}/?action=snapshot")
    
    print("\n" + "=" * 50)
    print("CAMERA SERVER RUNNING")
    print("=" * 50)
    print(f"Stream URL: http://localhost:{port}/?action=stream")
    print(f"Snapshot URL: http://localhost:{port}/?action=snapshot")
    print("\nLight sensor integration enabled:")
    print("  - Red border popup will appear when low light is detected")
    print("  - Warning: 'LOW LIGHT LEVEL DETECTED'")
    print("=" * 50)
    print("\nPress Ctrl+C to stop\n")
    
    # Main loop - update img_show with camera frames
    try:
        while True:
            ret, frame = camera.read()
            if ret:
                img_show = frame
            time.sleep(0.01)
    except KeyboardInterrupt:
        print("\nShutting down camera server...")
        camera.camera_close()
        if light_sensor:
            light_sensor.cleanup()
            print("Light sensor cleaned up")
        server.shutdown()
        print("Camera server stopped")


def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(
        description="TonyPi Camera Streaming Server with Light Sensor Integration",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Light Sensor Integration:
  When the light sensor detects low light levels, the camera stream will show:
  - Red border around the frame
  - "LOW LIGHT LEVEL DETECTED" warning popup
  - Warning message in console
  
  This matches the behavior of the FYP_Robot main.py
        """
    )
    parser.add_argument("--port", type=int, default=8081, help="HTTP server port (default: 8081)")
    parser.add_argument("--device", type=int, default=-1, help="Camera device index (-1 for auto)")
    parser.add_argument("--width", type=int, default=640, help="Frame width (default: 640)")
    parser.add_argument("--height", type=int, default=480, help="Frame height (default: 480)")
    parser.add_argument("--light-sensor-pin", type=int, default=24, help="GPIO pin for light sensor (default: 24)")
    
    args = parser.parse_args()
    
    start_camera_server(
        port=args.port,
        camera_device=args.device,
        resolution=(args.width, args.height),
        light_sensor_pin=args.light_sensor_pin
    )


if __name__ == "__main__":
    main()
