#!/usr/bin/env python3
"""
MJPEG Camera Streaming Server for TonyPi Robot
Captures frames from USB camera and serves them as MJPEG stream over HTTP.

Usage:
    python camera_stream.py [--port 8080]
    
Access the stream at:
    http://<tonypi-ip>:8080/?action=stream
    http://<tonypi-ip>:8080/?action=snapshot
"""

import time
import threading
import argparse
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

# Global frame storage
img_show = None
quality = (int(cv2.IMWRITE_JPEG_QUALITY) if CV2_AVAILABLE else 0, 70)


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
        """Background task for capturing frames."""
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
                        self.frame = frame
                    time.sleep(0.033)  # ~30fps
                    continue
                    
                if self.opened and self.cap is not None and self.cap.isOpened():
                    ret, frame_tmp = self.cap.read()
                    if ret:
                        frame = cv2.resize(frame_tmp, (self.width, self.height), 
                                          interpolation=cv2.INTER_NEAREST)
                        if self.flip:
                            self.frame = cv2.flip(frame, self.flip_param)
                        else:
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


def start_camera_server(port=8080, camera_device=-1, resolution=(640, 480)):
    """
    Start the MJPEG camera streaming server.
    
    Args:
        port: HTTP server port (default: 8080)
        camera_device: Camera device index (-1 for auto)
        resolution: Camera resolution tuple (width, height)
    """
    global img_show
    
    print(f"Starting camera stream server on port {port}...")
    print(f"Resolution: {resolution[0]}x{resolution[1]}")
    print(f"OpenCV available: {CV2_AVAILABLE}")
    
    # Initialize camera
    camera = Camera(resolution=resolution, device=camera_device)
    if not camera.camera_open():
        print("Warning: Camera failed to open, running with test pattern")
    
    # Start HTTP server
    server = ThreadedHTTPServer(('', port), MJPGHandler)
    server_thread = threading.Thread(target=server.serve_forever, daemon=True)
    server_thread.start()
    
    print(f"Camera server started!")
    print(f"Stream URL: http://localhost:{port}/?action=stream")
    print(f"Snapshot URL: http://localhost:{port}/?action=snapshot")
    
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
        server.shutdown()


def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(description="TonyPi Camera Streaming Server")
    parser.add_argument("--port", type=int, default=8080, help="HTTP server port")
    parser.add_argument("--device", type=int, default=-1, help="Camera device index")
    parser.add_argument("--width", type=int, default=640, help="Frame width")
    parser.add_argument("--height", type=int, default=480, help="Frame height")
    
    args = parser.parse_args()
    
    start_camera_server(
        port=args.port,
        camera_device=args.device,
        resolution=(args.width, args.height)
    )


if __name__ == "__main__":
    main()
