#!/usr/bin/env python3
"""
TonyPi Robot Client Module - MQTT Telemetry Integration

This module provides a simple interface for sending robot telemetry data
to a monitoring system via MQTT. Designed to be imported and used from main.py.

Usage:
    from modules.robot_client import RobotClient
    
    client = RobotClient(mqtt_broker="192.168.1.100")
    client.start()  # Connects in background
    
    # In your main loop:
    client.send_vision_data(detection_result)
    client.send_sensor_data(sensors)
    client.send_log("INFO", "Robot started")
    
    # When done:
    client.stop()
"""

import sys
import os
import json
import time
import threading
import platform
import socket
import psutil
from datetime import datetime
from typing import Dict, Any, Optional, Callable
import logging

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger("RobotClient")

# Try to import MQTT
try:
    import paho.mqtt.client as mqtt
    MQTT_AVAILABLE = True
except ImportError:
    MQTT_AVAILABLE = False
    logger.warning("paho-mqtt not installed. Run: pip install paho-mqtt")


class RobotClient:
    """
    Robot Client for sending telemetry to monitoring system.
    
    This is a lightweight wrapper designed to be used from main.py.
    It handles MQTT connection in the background and provides simple
    methods to send various types of data.
    """
    
    def __init__(
        self,
        mqtt_broker: str = "localhost",
        mqtt_port: int = 1883,
        robot_id: str = None,
        auto_telemetry: bool = True,
        telemetry_interval: float = 5.0
    ):
        """
        Initialize the Robot Client.
        
        Args:
            mqtt_broker: MQTT broker address
            mqtt_port: MQTT broker port  
            robot_id: Robot identifier (auto-generated if not provided)
            auto_telemetry: If True, automatically sends status/battery on interval
            telemetry_interval: Interval in seconds for auto telemetry
        """
        if not MQTT_AVAILABLE:
            raise ImportError("paho-mqtt is required. Install with: pip install paho-mqtt")
        
        self.mqtt_broker = mqtt_broker
        self.mqtt_port = mqtt_port
        self.auto_telemetry = auto_telemetry
        self.telemetry_interval = telemetry_interval
        
        # Generate robot ID from hostname
        if robot_id:
            self.robot_id = robot_id
        else:
            hostname = platform.node().lower().replace(" ", "_")
            self.robot_id = f"tonypi_{hostname}"
        
        # MQTT client
        self.client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=self.robot_id)
        self.is_connected = False
        self.running = False
        
        # Hardware detection
        self.hardware_available = self._detect_hardware()
        
        # State
        self.battery_level = 100.0
        self._last_battery_voltage = 12.6
        self.location = {"x": 0.0, "y": 0.0, "z": 0.0}
        self.status = "online"
        
        # MQTT Topics
        self.topics = {
            "sensors": f"tonypi/sensors/{self.robot_id}",
            "status": f"tonypi/status/{self.robot_id}",
            "location": f"tonypi/location",
            "battery": f"tonypi/battery",
            "commands": f"tonypi/commands/{self.robot_id}",
            "response": f"tonypi/commands/response",
            "servos": f"tonypi/servos/{self.robot_id}",
            "vision": f"tonypi/vision/{self.robot_id}",
            "logs": f"tonypi/logs/{self.robot_id}"
        }
        
        # Callbacks
        self.on_command_callback: Optional[Callable] = None
        
        # Setup MQTT callbacks
        self.client.on_connect = self._on_connect
        self.client.on_disconnect = self._on_disconnect
        self.client.on_message = self._on_message
        
        # Background thread
        self._telemetry_thread = None
        self._last_status_time = 0
        self._last_battery_time = 0
        
        logger.info(f"RobotClient initialized: {self.robot_id}")
    
    def _detect_hardware(self) -> bool:
        """Detect if running on TonyPi hardware."""
        try:
            # Check for Raspberry Pi
            if os.path.exists('/proc/device-tree/model'):
                with open('/proc/device-tree/model', 'r') as f:
                    if 'raspberry' in f.read().lower():
                        return True
            # Check for TonyPi SDK path
            if os.path.exists('/home/pi/TonyPi'):
                return True
        except:
            pass
        return False
    
    def _on_connect(self, client, userdata, flags, rc, properties=None):
        """MQTT connection callback."""
        if rc == 0:
            self.is_connected = True
            logger.info(f"Connected to MQTT broker: {self.mqtt_broker}:{self.mqtt_port}")
            
            # Subscribe to command topics
            client.subscribe(self.topics["commands"])
            client.subscribe("tonypi/commands/broadcast")
            
            # Send initial status
            self.send_status()
        else:
            logger.error(f"MQTT connection failed: rc={rc}")
    
    def _on_disconnect(self, client, userdata, flags=None, rc=None, properties=None):
        """MQTT disconnection callback."""
        self.is_connected = False
        return_code = rc if rc is not None else flags
        logger.warning(f"Disconnected from MQTT: rc={return_code}")
    
    def _on_message(self, client, userdata, msg):
        """Handle incoming MQTT messages."""
        try:
            payload = json.loads(msg.payload.decode())
            logger.info(f"Command received: {payload.get('type')}")
            
            if self.on_command_callback:
                self.on_command_callback(msg.topic, payload)
                
        except Exception as e:
            logger.error(f"Message handling error: {e}")
    
    def _telemetry_loop(self):
        """Background thread for auto telemetry."""
        while self.running:
            try:
                current_time = time.time()
                
                # Send status periodically
                if self.auto_telemetry and self.is_connected:
                    if current_time - self._last_status_time >= self.telemetry_interval:
                        self.send_status()
                        self._last_status_time = current_time
                    
                    if current_time - self._last_battery_time >= 30:
                        self.send_battery()
                        self._last_battery_time = current_time
                
                time.sleep(0.5)
            except Exception as e:
                logger.error(f"Telemetry loop error: {e}")
    
    # ==========================================
    # PUBLIC API
    # ==========================================
    
    def start(self) -> bool:
        """
        Start the robot client (connects to MQTT in background).
        
        Returns:
            True if connection initiated successfully
        """
        try:
            self.running = True
            self.client.connect(self.mqtt_broker, self.mqtt_port, 60)
            self.client.loop_start()
            
            # Wait for connection (with timeout)
            timeout = 5.0
            while not self.is_connected and timeout > 0:
                time.sleep(0.1)
                timeout -= 0.1
            
            if self.is_connected:
                # Start background telemetry thread
                self._telemetry_thread = threading.Thread(target=self._telemetry_loop, daemon=True)
                self._telemetry_thread.start()
                logger.info("Robot client started")
                return True
            else:
                logger.warning("MQTT connection timeout - running without monitoring")
                return False
                
        except Exception as e:
            logger.error(f"Failed to start client: {e}")
            return False
    
    def stop(self):
        """Stop the robot client and disconnect."""
        self.running = False
        self.status = "offline"
        
        if self.is_connected:
            self.send_status()
            time.sleep(0.5)
        
        self.client.loop_stop()
        self.client.disconnect()
        logger.info("Robot client stopped")
    
    def set_command_callback(self, callback: Callable):
        """
        Set callback for incoming commands.
        
        Args:
            callback: Function(topic, payload) to handle commands
        """
        self.on_command_callback = callback
    
    # ==========================================
    # TELEMETRY METHODS
    # ==========================================
    
    def send_vision_data(
        self,
        label: str = None,
        confidence: float = None,
        bbox: tuple = None,
        center_x: int = None,
        frame_width: int = 640,
        state: str = "UNKNOWN",
        is_locked: bool = False,
        nav_cmd: str = None,
        error: float = None
    ):
        """
        Send vision detection data to monitoring system.
        
        Args:
            label: Detected object class name
            confidence: Detection confidence (0-1)
            bbox: Bounding box (x1, y1, x2, y2)
            center_x: Center X coordinate of detection
            frame_width: Camera frame width
            state: Current robot state
            is_locked: Whether target is locked
            nav_cmd: Navigation command (TURN_LEFT, TURN_RIGHT, LOCKED)
            error: Position error from center
        """
        if not self.is_connected:
            return
        
        try:
            data = {
                "robot_id": self.robot_id,
                "timestamp": datetime.now().isoformat(),
                "detection": label is not None,
                "label": label,
                "confidence": confidence,
                "bbox": list(bbox) if bbox else None,
                "center_x": center_x,
                "frame_width": frame_width,
                "state": state,
                "is_locked": is_locked,
                "navigation_command": nav_cmd,
                "error": error
            }
            
            self.client.publish(self.topics["vision"], json.dumps(data))
            
        except Exception as e:
            logger.error(f"Error sending vision data: {e}")
    
    def send_sensor_data(self, sensors: Dict[str, float]):
        """
        Send sensor data to monitoring system.
        
        Args:
            sensors: Dictionary of sensor_name: value pairs
                     e.g. {"ultrasonic_distance": 25.5, "light_level": 80}
        """
        if not self.is_connected:
            return
        
        try:
            for sensor_name, value in sensors.items():
                if isinstance(value, str):
                    continue  # Skip string values
                    
                data = {
                    "robot_id": self.robot_id,
                    "sensor_type": sensor_name,
                    "value": value,
                    "timestamp": datetime.now().isoformat(),
                    "unit": self._get_sensor_unit(sensor_name)
                }
                self.client.publish(self.topics["sensors"], json.dumps(data))
                
        except Exception as e:
            logger.error(f"Error sending sensor data: {e}")
    
    def send_status(self, custom_data: Dict = None):
        """
        Send robot status update with system metrics.
        
        Args:
            custom_data: Optional additional data to include
        """
        if not self.is_connected:
            return
        
        try:
            system_info = self._get_system_info()
            ip_address = self._get_local_ip()
            
            data = {
                "robot_id": self.robot_id,
                "status": self.status,
                "timestamp": datetime.now().isoformat(),
                "system_info": system_info,
                "hardware_available": self.hardware_available,
                "ip_address": ip_address,
                "camera_url": f"http://{ip_address}:8081/?action=stream"
            }
            
            if custom_data:
                data.update(custom_data)
            
            self.client.publish(self.topics["status"], json.dumps(data))
            
        except Exception as e:
            logger.error(f"Error sending status: {e}")
    
    def send_battery(self, percentage: float = None, voltage: float = None):
        """
        Send battery status.
        
        Args:
            percentage: Battery percentage (0-100)
            voltage: Battery voltage in volts
        """
        if not self.is_connected:
            return
        
        try:
            if percentage is None:
                percentage = self.battery_level
            if voltage is None:
                voltage = self._last_battery_voltage
            
            data = {
                "robot_id": self.robot_id,
                "percentage": round(percentage, 1),
                "voltage": round(voltage, 2),
                "charging": False,
                "timestamp": datetime.now().isoformat()
            }
            
            self.client.publish(self.topics["battery"], json.dumps(data))
            
        except Exception as e:
            logger.error(f"Error sending battery: {e}")
    
    def send_servo_data(self, servo_data: Dict[str, Any]):
        """
        Send servo status data.
        
        Args:
            servo_data: Dictionary of servo data
        """
        if not self.is_connected:
            return
        
        try:
            data = {
                "robot_id": self.robot_id,
                "servos": servo_data,
                "servo_count": len(servo_data),
                "timestamp": datetime.now().isoformat()
            }
            
            self.client.publish(self.topics["servos"], json.dumps(data))
            
        except Exception as e:
            logger.error(f"Error sending servo data: {e}")
    
    def send_location(self, x: float, y: float, z: float = 0.0):
        """
        Send robot location/position.
        
        Args:
            x, y, z: Position coordinates
        """
        if not self.is_connected:
            return
        
        try:
            self.location = {"x": x, "y": y, "z": z}
            
            data = {
                "robot_id": self.robot_id,
                "x": x,
                "y": y,
                "z": z,
                "timestamp": datetime.now().isoformat()
            }
            
            self.client.publish(self.topics["location"], json.dumps(data))
            
        except Exception as e:
            logger.error(f"Error sending location: {e}")
    
    def send_log(self, level: str, message: str, source: str = "main"):
        """
        Send log message to monitoring system.
        
        Args:
            level: Log level (DEBUG, INFO, WARNING, ERROR)
            message: Log message
            source: Source module name
        """
        if not self.is_connected:
            return
        
        try:
            data = {
                "robot_id": self.robot_id,
                "timestamp": datetime.now().isoformat(),
                "level": level.upper(),
                "message": message,
                "source": source
            }
            
            self.client.publish(self.topics["logs"], json.dumps(data))
            
        except Exception as e:
            logger.error(f"Error sending log: {e}")
    
    def send_command_response(self, command_id: str, success: bool, message: str, data: Dict = None):
        """
        Send response to a command.
        
        Args:
            command_id: ID of the command being responded to
            success: Whether command succeeded
            message: Response message
            data: Optional additional data
        """
        if not self.is_connected:
            return
        
        try:
            response = {
                "robot_id": self.robot_id,
                "command_id": command_id,
                "timestamp": datetime.now().isoformat(),
                "success": success,
                "message": message
            }
            
            if data:
                response["data"] = data
            
            self.client.publish(self.topics["response"], json.dumps(response))
            
        except Exception as e:
            logger.error(f"Error sending command response: {e}")
    
    # ==========================================
    # HELPER METHODS
    # ==========================================
    
    def _get_system_info(self) -> Dict[str, Any]:
        """Get system metrics."""
        try:
            cpu_temp = self._get_cpu_temperature()
            return {
                "platform": platform.platform(),
                "cpu_percent": psutil.cpu_percent(interval=None),
                "memory_percent": psutil.virtual_memory().percent,
                "disk_usage": psutil.disk_usage('/').percent if os.path.exists('/') else 0,
                "temperature": cpu_temp,
                "cpu_temperature": cpu_temp,
                "uptime": time.time() - psutil.boot_time(),
                "hardware_mode": self.hardware_available
            }
        except Exception as e:
            logger.error(f"Error getting system info: {e}")
            return {"error": str(e)}
    
    def _get_cpu_temperature(self) -> float:
        """Get CPU temperature."""
        try:
            # Raspberry Pi
            if os.path.exists('/sys/class/thermal/thermal_zone0/temp'):
                with open('/sys/class/thermal/thermal_zone0/temp', 'r') as f:
                    return float(f.read()) / 1000.0
        except:
            pass
        # Fallback: simulated
        return 45.0 + (time.time() % 10)
    
    def _get_local_ip(self) -> str:
        """Get local IP address."""
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(("8.8.8.8", 80))
            ip = s.getsockname()[0]
            s.close()
            return ip
        except:
            return "192.168.149.1"
    
    def _get_sensor_unit(self, sensor_name: str) -> str:
        """Get unit for sensor type."""
        units = {
            "accelerometer_x": "m/s²",
            "accelerometer_y": "m/s²",
            "accelerometer_z": "m/s²",
            "gyroscope_x": "°/s",
            "gyroscope_y": "°/s",
            "gyroscope_z": "°/s",
            "ultrasonic_distance": "cm",
            "cpu_temperature": "°C",
            "light_level": "%",
            "light_sensor_dark": "bool"
        }
        return units.get(sensor_name, "")
    
    @property
    def connected(self) -> bool:
        """Check if connected to MQTT broker."""
        return self.is_connected


# Convenience function for quick setup
def create_client(broker: str = None, robot_id: str = None) -> RobotClient:
    """
    Create and start a robot client.
    
    Args:
        broker: MQTT broker address (default: localhost or MQTT_BROKER env)
        robot_id: Robot ID (default: auto-generated)
    
    Returns:
        Connected RobotClient instance
    """
    if broker is None:
        broker = os.getenv("MQTT_BROKER", "localhost")
    
    port = int(os.getenv("MQTT_PORT", 1883))
    
    client = RobotClient(
        mqtt_broker=broker,
        mqtt_port=port,
        robot_id=robot_id
    )
    client.start()
    return client
