#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
TonyPi Servo Controller Module
==============================
Low-level servo control interface for the TonyPi humanoid robot.
This module handles direct communication with bus servos and PWM servos,
including angle-to-pulse conversion, coordinated multi-servo movement,
and motion interpolation.

=== HOW THIS MODULE IS USED FOR ACTION GROUPS ===

1. The kinematics module calculates joint angles (in degrees)
2. This module converts angles to servo pulse values (500-2500μs)
3. The pulse values control the actual servo motor positions
4. These values are stored in .d6a files for action playback

KEY FUNCTION: angle_to_pulse(angle, servo_name)
    - Converts degrees to microseconds
    - Formula: pulse = min_pulse + (angle / angle_range) × pulse_range
    - Example: 90° on head_pan → 1500μs (center position)

SERVO PULSE RANGE:
    - 500μs  = Minimum position
    - 1500μs = Center/neutral position
    - 2500μs = Maximum position

Hardware Interface:
- Bus Servos (ID 1-18): Serial bus communication, position feedback
- PWM Servos (ID 1-2): Pulse width modulation, head pan/tilt

Author: TonyPi FYP Project
"""

import time
import math
from typing import Dict, List, Tuple, Optional
from dataclasses import dataclass
from enum import Enum

# Hardware imports (TonyPi SDK)
try:
    import hiwonder.ros_robot_controller_sdk as rrc
    from hiwonder.Controller import Controller
    HARDWARE_AVAILABLE = True
except ImportError:
    print("[ServoController] Hardware SDK not available - simulation mode")
    HARDWARE_AVAILABLE = False


# =============================================================================
# SERVO CONFIGURATION
# =============================================================================

class ServoType(Enum):
    """Type of servo motor"""
    BUS = "bus"      # Serial bus servo (position feedback)
    PWM = "pwm"      # PWM servo (no feedback)


@dataclass
class ServoConfig:
    """Configuration for a single servo"""
    servo_id: int
    servo_type: ServoType
    name: str
    min_pulse: int = 500      # Minimum pulse width (μs)
    max_pulse: int = 2500     # Maximum pulse width (μs)
    min_angle: float = 0.0    # Minimum angle (degrees)
    max_angle: float = 180.0  # Maximum angle (degrees)
    default_pulse: int = 1500 # Neutral position
    inverted: bool = False    # Invert direction


# TonyPi Servo Mapping
# Based on hardware specification sheet
SERVO_MAP: Dict[str, ServoConfig] = {
    # ========== HEAD (PWM Servos) ==========
    'head_tilt': ServoConfig(
        servo_id=1, servo_type=ServoType.PWM,
        name='Head Tilt', min_pulse=500, max_pulse=2500,
        min_angle=-45, max_angle=45, default_pulse=1500
    ),
    'head_pan': ServoConfig(
        servo_id=2, servo_type=ServoType.PWM,
        name='Head Pan', min_pulse=500, max_pulse=2500,
        min_angle=-90, max_angle=90, default_pulse=1500
    ),
    
    # ========== LEFT ARM (Bus Servos 5-8) ==========
    'left_shoulder': ServoConfig(
        servo_id=5, servo_type=ServoType.BUS,
        name='Left Shoulder', min_angle=0, max_angle=180, default_pulse=500
    ),
    'left_upper_arm': ServoConfig(
        servo_id=6, servo_type=ServoType.BUS,
        name='Left Upper Arm', min_angle=0, max_angle=180, default_pulse=500
    ),
    'left_elbow': ServoConfig(
        servo_id=7, servo_type=ServoType.BUS,
        name='Left Elbow', min_angle=0, max_angle=135, default_pulse=500
    ),
    'left_wrist': ServoConfig(
        servo_id=8, servo_type=ServoType.BUS,
        name='Left Wrist', min_angle=0, max_angle=180, default_pulse=500
    ),
    
    # ========== RIGHT ARM (Bus Servos 9-12) ==========
    'right_wrist': ServoConfig(
        servo_id=9, servo_type=ServoType.BUS,
        name='Right Wrist', min_angle=0, max_angle=180, default_pulse=500
    ),
    'right_elbow': ServoConfig(
        servo_id=10, servo_type=ServoType.BUS,
        name='Right Elbow', min_angle=0, max_angle=135, default_pulse=500
    ),
    'right_upper_arm': ServoConfig(
        servo_id=11, servo_type=ServoType.BUS,
        name='Right Upper Arm', min_angle=0, max_angle=180, default_pulse=500
    ),
    'right_shoulder': ServoConfig(
        servo_id=12, servo_type=ServoType.BUS,
        name='Right Shoulder', min_angle=0, max_angle=180, default_pulse=500
    ),
    
    # ========== LEFT LEG (Bus Servos 13-15) ==========
    'left_hip': ServoConfig(
        servo_id=13, servo_type=ServoType.BUS,
        name='Left Hip', min_angle=-45, max_angle=90, default_pulse=500
    ),
    'left_knee': ServoConfig(
        servo_id=14, servo_type=ServoType.BUS,
        name='Left Knee', min_angle=0, max_angle=150, default_pulse=500
    ),
    'left_ankle': ServoConfig(
        servo_id=15, servo_type=ServoType.BUS,
        name='Left Ankle', min_angle=-45, max_angle=45, default_pulse=500
    ),
    
    # ========== RIGHT LEG (Bus Servos 16, 1-2 or extended) ==========
    'right_hip': ServoConfig(
        servo_id=16, servo_type=ServoType.BUS,
        name='Right Hip', min_angle=-45, max_angle=90, default_pulse=500
    ),
    'right_knee': ServoConfig(
        servo_id=3, servo_type=ServoType.BUS,
        name='Right Knee', min_angle=0, max_angle=150, default_pulse=500
    ),
    'right_ankle': ServoConfig(
        servo_id=4, servo_type=ServoType.BUS,
        name='Right Ankle', min_angle=-45, max_angle=45, default_pulse=500
    ),
}


# =============================================================================
# SERVO CONTROLLER CLASS
# =============================================================================

class ServoController:
    """
    Low-level servo controller for TonyPi robot.
    
    Provides:
    - Individual servo control
    - Coordinated multi-servo movement
    - Smooth motion interpolation
    - Angle-to-pulse conversion
    """
    
    def __init__(self):
        """Initialize servo controller and hardware connection"""
        self.servo_map = SERVO_MAP
        self.current_positions: Dict[str, int] = {}  # Current pulse positions
        
        # Initialize hardware
        if HARDWARE_AVAILABLE:
            self.board = rrc.Board()
            self.controller = Controller(self.board)
            print("[ServoController] Hardware initialized successfully")
        else:
            self.board = None
            self.controller = None
            print("[ServoController] Running in simulation mode")
        
        # Initialize all servos to default positions
        self._initialize_default_positions()
    
    def _initialize_default_positions(self):
        """Set all servos to their default (neutral) positions"""
        for name, config in self.servo_map.items():
            self.current_positions[name] = config.default_pulse
    
    # =========================================================================
    # ANGLE/PULSE CONVERSION
    # =========================================================================
    
    def angle_to_pulse(self, angle: float, servo_name: str) -> int:
        """
        Convert angle (degrees) to pulse width (microseconds).
        
        === SIMPLE EXPLANATION ===
        
        Servos don't understand "30 degrees" - they need a pulse signal.
        This function converts human-readable angles to servo signals.
        
        FORMULA:
            pulse = min_pulse + (angle - min_angle) / (max_angle - min_angle) × (max_pulse - min_pulse)
        
        EXAMPLE (head_pan servo):
            - Angle range: -90° to +90°
            - Pulse range: 500μs to 2500μs
            - Input: 0° → Output: 1500μs (center)
            - Input: 45° → Output: 2000μs
        
        This is used by action_group_generator.py to convert
        kinematics angles into servo control values for .d6a files.
        
        Args:
            angle: Target angle in degrees
            servo_name: Name of the servo (key in SERVO_MAP)
            
        Returns:
            Pulse width in microseconds (500-2500 typical)
        """
        config = self.servo_map.get(servo_name)
        if not config:
            raise ValueError(f"Unknown servo: {servo_name}")
        
        # Clamp angle to valid range
        angle = max(config.min_angle, min(config.max_angle, angle))
        
        # Linear interpolation
        angle_range = config.max_angle - config.min_angle
        pulse_range = config.max_pulse - config.min_pulse
        
        normalized = (angle - config.min_angle) / angle_range
        pulse = int(config.min_pulse + normalized * pulse_range)
        
        # Invert if necessary
        if config.inverted:
            pulse = config.max_pulse - (pulse - config.min_pulse)
        
        return pulse
    
    def pulse_to_angle(self, pulse: int, servo_name: str) -> float:
        """
        Convert pulse width (microseconds) to angle (degrees).
        
        Inverse of angle_to_pulse.
        """
        config = self.servo_map.get(servo_name)
        if not config:
            raise ValueError(f"Unknown servo: {servo_name}")
        
        # Handle inversion
        if config.inverted:
            pulse = config.max_pulse - (pulse - config.min_pulse)
        
        # Linear interpolation
        pulse_range = config.max_pulse - config.min_pulse
        angle_range = config.max_angle - config.min_angle
        
        normalized = (pulse - config.min_pulse) / pulse_range
        angle = config.min_angle + normalized * angle_range
        
        return angle
    
    # =========================================================================
    # INDIVIDUAL SERVO CONTROL
    # =========================================================================
    
    def set_servo_pulse(self, servo_name: str, pulse: int, duration_ms: int = 300):
        """
        Set a single servo to a specific pulse width.
        
        Args:
            servo_name: Name of the servo
            pulse: Target pulse width in microseconds
            duration_ms: Time to reach target position
        """
        config = self.servo_map.get(servo_name)
        if not config:
            print(f"[ServoController] Unknown servo: {servo_name}")
            return
        
        # Clamp pulse to valid range
        pulse = max(config.min_pulse, min(config.max_pulse, pulse))
        
        # Send command to hardware
        if self.controller:
            if config.servo_type == ServoType.BUS:
                self.controller.set_bus_servo_pulse(
                    config.servo_id, pulse, duration_ms
                )
            else:  # PWM
                self.controller.set_pwm_servo_pulse(
                    config.servo_id, pulse, duration_ms
                )
        
        # Update internal state
        self.current_positions[servo_name] = pulse
        
        print(f"[Servo] {servo_name} (ID:{config.servo_id}) -> {pulse}μs ({duration_ms}ms)")
    
    def set_servo_angle(self, servo_name: str, angle: float, duration_ms: int = 300):
        """
        Set a single servo to a specific angle.
        
        Args:
            servo_name: Name of the servo
            angle: Target angle in degrees
            duration_ms: Time to reach target position
        """
        pulse = self.angle_to_pulse(angle, servo_name)
        self.set_servo_pulse(servo_name, pulse, duration_ms)
    
    # =========================================================================
    # COORDINATED MULTI-SERVO MOVEMENT
    # =========================================================================
    
    def set_multiple_servos(self, 
                            positions: Dict[str, int], 
                            duration_ms: int = 300):
        """
        Move multiple servos simultaneously.
        
        All servos start and finish movement at the same time,
        regardless of the distance each needs to travel.
        
        Args:
            positions: Dictionary of {servo_name: pulse_value}
            duration_ms: Time for all servos to complete movement
        """
        print(f"[ServoController] Coordinated move ({len(positions)} servos, {duration_ms}ms)")
        
        for servo_name, pulse in positions.items():
            config = self.servo_map.get(servo_name)
            if not config:
                continue
            
            pulse = max(config.min_pulse, min(config.max_pulse, pulse))
            
            if self.controller:
                if config.servo_type == ServoType.BUS:
                    self.controller.set_bus_servo_pulse(
                        config.servo_id, pulse, duration_ms
                    )
                else:
                    self.controller.set_pwm_servo_pulse(
                        config.servo_id, pulse, duration_ms
                    )
            
            self.current_positions[servo_name] = pulse
    
    def set_leg_position(self,
                         is_left: bool,
                         hip_angle: float,
                         knee_angle: float,
                         ankle_angle: float,
                         duration_ms: int = 300):
        """
        Set all joints of one leg simultaneously.
        
        Converts angles to pulses and sends coordinated command.
        """
        prefix = 'left' if is_left else 'right'
        
        positions = {
            f'{prefix}_hip': self.angle_to_pulse(hip_angle, f'{prefix}_hip'),
            f'{prefix}_knee': self.angle_to_pulse(knee_angle, f'{prefix}_knee'),
            f'{prefix}_ankle': self.angle_to_pulse(ankle_angle, f'{prefix}_ankle'),
        }
        
        self.set_multiple_servos(positions, duration_ms)
    
    # =========================================================================
    # SMOOTH MOTION INTERPOLATION
    # =========================================================================
    
    def interpolate_movement(self,
                              servo_name: str,
                              start_pulse: int,
                              end_pulse: int,
                              duration_ms: int,
                              steps: int = 20,
                              easing: str = 'ease_in_out'):
        """
        Smooth interpolated movement between two positions.
        
        Uses easing functions for natural-looking motion.
        
        Args:
            servo_name: Target servo
            start_pulse: Starting position
            end_pulse: Target position
            duration_ms: Total movement time
            steps: Number of intermediate positions
            easing: Easing function ('linear', 'ease_in', 'ease_out', 'ease_in_out')
        """
        step_delay = duration_ms / steps / 1000.0  # Convert to seconds
        
        for i in range(steps + 1):
            t = i / steps  # Progress from 0 to 1
            
            # Apply easing function
            if easing == 'ease_in':
                t = t * t
            elif easing == 'ease_out':
                t = 1 - (1 - t) * (1 - t)
            elif easing == 'ease_in_out':
                t = 3 * t * t - 2 * t * t * t  # Smoothstep
            # else: linear (no change to t)
            
            # Interpolate position
            current_pulse = int(start_pulse + t * (end_pulse - start_pulse))
            
            self.set_servo_pulse(servo_name, current_pulse, int(step_delay * 1000))
            time.sleep(step_delay)
    
    # =========================================================================
    # PREDEFINED POSES
    # =========================================================================
    
    def stand(self, duration_ms: int = 500):
        """Move robot to standing position"""
        print("[ServoController] Moving to STAND position")
        
        # Standing pose - all legs straight, arms at sides
        stand_pose = {
            # Left leg
            'left_hip': 1500,
            'left_knee': 1500,
            'left_ankle': 1500,
            # Right leg
            'right_hip': 1500,
            'right_knee': 1500,
            'right_ankle': 1500,
            # Left arm
            'left_shoulder': 1500,
            'left_elbow': 1500,
            # Right arm
            'right_shoulder': 1500,
            'right_elbow': 1500,
            # Head
            'head_pan': 1500,
            'head_tilt': 1500,
        }
        
        self.set_multiple_servos(stand_pose, duration_ms)
    
    def crouch(self, depth: float = 0.5, duration_ms: int = 500):
        """
        Crouch down by bending knees.
        
        Args:
            depth: Crouch depth (0.0 = standing, 1.0 = full crouch)
        """
        print(f"[ServoController] Crouching (depth: {depth})")
        
        # Calculate knee bend based on depth
        knee_bend = int(500 + depth * 400)  # 500 to 900 pulse range
        
        crouch_pose = {
            'left_hip': 1500 + int(depth * 200),
            'left_knee': knee_bend,
            'left_ankle': 1500 - int(depth * 200),
            'right_hip': 1500 + int(depth * 200),
            'right_knee': knee_bend,
            'right_ankle': 1500 - int(depth * 200),
        }
        
        self.set_multiple_servos(crouch_pose, duration_ms)
    
    # =========================================================================
    # DEBUGGING & STATUS
    # =========================================================================
    
    def get_servo_status(self, servo_name: str) -> Optional[Dict]:
        """Get current status of a servo"""
        config = self.servo_map.get(servo_name)
        if not config:
            return None
        
        current_pulse = self.current_positions.get(servo_name, config.default_pulse)
        current_angle = self.pulse_to_angle(current_pulse, servo_name)
        
        return {
            'name': servo_name,
            'id': config.servo_id,
            'type': config.servo_type.value,
            'current_pulse': current_pulse,
            'current_angle': current_angle,
            'limits': (config.min_angle, config.max_angle)
        }
    
    def print_all_status(self):
        """Print status of all servos"""
        print("\n=== Servo Status ===")
        print(f"{'Name':<20} {'ID':<4} {'Pulse':<8} {'Angle':<10}")
        print("-" * 45)
        
        for name in sorted(self.servo_map.keys()):
            status = self.get_servo_status(name)
            if status:
                print(f"{name:<20} {status['id']:<4} {status['current_pulse']:<8} "
                      f"{status['current_angle']:.1f}°")


# =============================================================================
# GAIT CONTROLLER (WALKING PATTERNS)
# =============================================================================

class GaitController:
    """
    Generates walking gait patterns by coordinating servo movements.
    
    Walking cycle phases:
    1. Right stance, left swing (lift left foot, move forward)
    2. Double support (both feet on ground)
    3. Left stance, right swing (lift right foot, move forward)
    4. Double support (both feet on ground)
    """
    
    def __init__(self, servo_controller: ServoController):
        self.servo = servo_controller
        self.step_length = 3.0      # cm
        self.step_height = 2.0      # cm
        self.step_duration = 400    # ms per step
    
    def walk_forward_step(self):
        """Execute one forward walking step"""
        print("[Gait] Walking forward - one step")
        
        # Phase 1: Shift weight to right leg, lift left foot
        self._shift_weight_right()
        time.sleep(0.1)
        
        # Phase 2: Swing left leg forward
        self._swing_left_forward()
        time.sleep(self.step_duration / 1000)
        
        # Phase 3: Shift weight to left leg, lift right foot
        self._shift_weight_left()
        time.sleep(0.1)
        
        # Phase 4: Swing right leg forward
        self._swing_right_forward()
        time.sleep(self.step_duration / 1000)
    
    def _shift_weight_right(self):
        """Shift center of mass over right foot"""
        self.servo.set_multiple_servos({
            'left_hip': 1400,    # Tilt body right
            'right_hip': 1600,
            'left_ankle': 1400,  # Compensate ankle
            'right_ankle': 1600,
        }, 200)
    
    def _shift_weight_left(self):
        """Shift center of mass over left foot"""
        self.servo.set_multiple_servos({
            'left_hip': 1600,
            'right_hip': 1400,
            'left_ankle': 1600,
            'right_ankle': 1400,
        }, 200)
    
    def _swing_left_forward(self):
        """Lift and swing left leg forward"""
        # Lift foot
        self.servo.set_multiple_servos({
            'left_hip': 1700,    # Flex hip forward
            'left_knee': 1200,   # Bend knee to lift foot
            'left_ankle': 1300,  # Point toe down
        }, 150)
        
        time.sleep(0.15)
        
        # Extend forward
        self.servo.set_multiple_servos({
            'left_hip': 1650,
            'left_knee': 1500,   # Straighten knee
            'left_ankle': 1500,  # Level foot
        }, 200)
    
    def _swing_right_forward(self):
        """Lift and swing right leg forward"""
        # Lift foot
        self.servo.set_multiple_servos({
            'right_hip': 1700,
            'right_knee': 1200,
            'right_ankle': 1300,
        }, 150)
        
        time.sleep(0.15)
        
        # Extend forward
        self.servo.set_multiple_servos({
            'right_hip': 1650,
            'right_knee': 1500,
            'right_ankle': 1500,
        }, 200)
    
    def turn_left(self):
        """Turn robot left by rotating on the spot"""
        print("[Gait] Turning left")
        
        self.servo.set_multiple_servos({
            'left_hip': 1400,    # Rotate left hip back
            'right_hip': 1600,   # Rotate right hip forward
        }, 300)
        
        time.sleep(0.3)
        
        # Return to neutral
        self.servo.set_multiple_servos({
            'left_hip': 1500,
            'right_hip': 1500,
        }, 300)
    
    def turn_right(self):
        """Turn robot right by rotating on the spot"""
        print("[Gait] Turning right")
        
        self.servo.set_multiple_servos({
            'left_hip': 1600,
            'right_hip': 1400,
        }, 300)
        
        time.sleep(0.3)
        
        self.servo.set_multiple_servos({
            'left_hip': 1500,
            'right_hip': 1500,
        }, 300)


# =============================================================================
# USAGE EXAMPLE
# =============================================================================

if __name__ == "__main__":
    print("=== TonyPi Servo Controller Demo ===\n")
    
    # Initialize controller
    servo = ServoController()
    
    # Show angle to pulse conversion
    print("Angle to Pulse Conversion Examples:")
    print(f"  head_pan at 0°   -> {servo.angle_to_pulse(0, 'head_pan')}μs")
    print(f"  head_pan at 45°  -> {servo.angle_to_pulse(45, 'head_pan')}μs")
    print(f"  head_pan at 90°  -> {servo.angle_to_pulse(90, 'head_pan')}μs")
    print(f"  left_knee at 0°  -> {servo.angle_to_pulse(0, 'left_knee')}μs")
    print(f"  left_knee at 90° -> {servo.angle_to_pulse(90, 'left_knee')}μs")
    
    # Show all servo status
    servo.print_all_status()
    
    # Demo coordinated movement (simulation only - won't move real robot)
    print("\n--- Demo: Standing Position ---")
    # servo.stand()  # Uncomment to test on real robot
    
    # Demo gait controller
    gait = GaitController(servo)
    print("\n--- Demo: Walk Forward Step ---")
    # gait.walk_forward_step()  # Uncomment to test on real robot
