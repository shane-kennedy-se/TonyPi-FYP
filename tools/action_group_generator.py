#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
================================================================================
TonyPi Action Group Generator
================================================================================
This tool generates .d6a action group files for the TonyPi robot.

HOW TO USE:
-----------
1. Define your keyframes with servo IDs, pulses, and timing
2. Run this script to generate the .d6a file
3. The file is saved to the ../actions/ folder
4. Use AGC.runActionGroup("YourActionName") to execute it

SERVO CONFIGURATION:
--------------------
Bus Servos (ID 1-18): Body joints + clamps
    - ID 1-4: Right leg (hip, knee, ankle) + spare
    - ID 5-8: Left arm (shoulder, upper arm, elbow, wrist)
    - ID 9-12: Right arm (wrist, elbow, upper arm, shoulder)
    - ID 13-15: Left leg (hip, knee, ankle)
    - ID 16: Right hip
    - ID 17-18: Clamps/grippers

PWM Servos (ID 1-2): Head control
    - ID 1: Head tilt (up/down)
    - ID 2: Head pan (left/right)

RELATIONSHIP TO OTHER MODULES:
------------------------------
- servo_controller.py: Provides angle-to-pulse conversion
- kinematics.py: Calculates joint angles from target positions
- This file: Uses those calculations to generate .d6a action files

================================================================================
"""

import os
import sys
import struct
from typing import Dict, List, Tuple

# Add modules path for imports
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'modules'))

# Import servo controller for angle-to-pulse conversion
try:
    from servo_controller import ServoController, SERVO_MAP
    SERVO_CONTROLLER_AVAILABLE = True
except ImportError:
    print("[INFO] servo_controller.py not found - angle conversion disabled")
    SERVO_CONTROLLER_AVAILABLE = False
    SERVO_MAP = {}

# Import kinematics for calculating angles from positions
try:
    from kinematics import LegIK, ArmIK, Point3D, RobotDimensions
    KINEMATICS_AVAILABLE = True
except ImportError:
    print("[INFO] kinematics.py not found - IK calculations disabled")
    KINEMATICS_AVAILABLE = False


# =============================================================================
# SERVO ID REFERENCE TABLE
# =============================================================================
"""
TonyPi Robot Servo Mapping:

BUS SERVOS (ID 1-18):
┌────────┬──────────────────────┬─────────────────────────────────┐
│ ID     │ Joint Name           │ Description                     │
├────────┼──────────────────────┼─────────────────────────────────┤
│ 1      │ Head Tilt (PWM)      │ Up/Down movement                │
│ 2      │ Head Pan (PWM)       │ Left/Right rotation             │
├────────┼──────────────────────┼─────────────────────────────────┤
│ 3      │ Right Knee           │ Right leg knee bend             │
│ 4      │ Right Ankle          │ Right leg ankle                 │
├────────┼──────────────────────┼─────────────────────────────────┤
│ 5      │ Left Shoulder        │ Left arm shoulder               │
│ 6      │ Left Upper Arm       │ Left arm upper section          │
│ 7      │ Left Elbow           │ Left arm elbow bend             │
│ 8      │ Left Wrist           │ Left hand/gripper wrist         │
├────────┼──────────────────────┼─────────────────────────────────┤
│ 9      │ Right Wrist          │ Right hand/gripper wrist        │
│ 10     │ Right Elbow          │ Right arm elbow bend            │
│ 11     │ Right Upper Arm      │ Right arm upper section         │
│ 12     │ Right Shoulder       │ Right arm shoulder              │
├────────┼──────────────────────┼─────────────────────────────────┤
│ 13     │ Left Hip             │ Left leg hip rotation           │
│ 14     │ Left Knee            │ Left leg knee bend              │
│ 15     │ Left Ankle           │ Left leg ankle                  │
├────────┼──────────────────────┼─────────────────────────────────┤
│ 16     │ Right Hip            │ Right leg hip rotation          │
│ 17     │ Left Clamp           │ Left gripper open/close         │
│ 18     │ Right Clamp          │ Right gripper open/close        │
└────────┴──────────────────────┴─────────────────────────────────┘

PULSE RANGE: 500-2500 microseconds (μs)
    - 500μs  = Minimum position
    - 1500μs = Center/neutral position
    - 2500μs = Maximum position
"""

# Servo ID constants for easy reference
SERVO_IDS = {
    # PWM Servos (Head)
    'head_tilt': 1,
    'head_pan': 2,
    
    # Right Leg
    'right_knee': 3,
    'right_ankle': 4,
    
    # Left Arm
    'left_shoulder': 5,
    'left_upper_arm': 6,
    'left_elbow': 7,
    'left_wrist': 8,
    
    # Right Arm
    'right_wrist': 9,
    'right_elbow': 10,
    'right_upper_arm': 11,
    'right_shoulder': 12,
    
    # Left Leg
    'left_hip': 13,
    'left_knee': 14,
    'left_ankle': 15,
    
    # Right Leg (continued)
    'right_hip': 16,
    
    # Clamps/Grippers
    'left_clamp': 17,
    'right_clamp': 18,
}


# =============================================================================
# KEYFRAME CLASS
# =============================================================================

class Keyframe:
    """
    A single keyframe in an action sequence.
    
    Attributes:
        duration_ms: Time to transition to this pose (milliseconds)
        servos: Dictionary of {servo_id: pulse_value}
    """
    
    def __init__(self, duration_ms: int = 500):
        """
        Create a new keyframe.
        
        Args:
            duration_ms: Transition time from previous keyframe (in milliseconds)
                         This is how long it takes to move FROM the previous
                         keyframe TO this one.
        """
        self.duration_ms = duration_ms
        self.servos: Dict[int, int] = {}  # {servo_id: pulse_value}
    
    def set_servo(self, servo_id: int, pulse: int):
        """
        Set a servo position using servo ID and pulse value.
        
        Args:
            servo_id: Servo ID (1-18 for bus servos, 1-2 for PWM)
            pulse: Pulse width in microseconds (500-2500)
        
        Example:
            kf.set_servo(13, 1500)  # Left hip at center position
            kf.set_servo(14, 1200)  # Left knee bent
        """
        # Validate servo ID (1-18 for bus, 1-2 for PWM - same IDs overlap)
        if servo_id < 1 or servo_id > 18:
            print(f"[WARNING] Servo ID {servo_id} out of range (1-18)")
        
        # Clamp pulse to valid range
        pulse = max(500, min(2500, pulse))
        self.servos[servo_id] = pulse
    
    def set_servo_by_name(self, servo_name: str, pulse: int):
        """
        Set a servo position using servo name instead of ID.
        
        Args:
            servo_name: Name from SERVO_IDS (e.g., 'left_hip', 'right_clamp')
            pulse: Pulse width in microseconds (500-2500)
        
        Example:
            kf.set_servo_by_name('left_hip', 1500)
            kf.set_servo_by_name('left_clamp', 1200)
        """
        servo_id = SERVO_IDS.get(servo_name.lower())
        if servo_id is None:
            print(f"[ERROR] Unknown servo name: {servo_name}")
            print(f"        Valid names: {list(SERVO_IDS.keys())}")
            return
        self.set_servo(servo_id, pulse)
    
    def set_servo_angle(self, servo_name: str, angle: float, controller):
        """
        Set a servo position using angle (requires servo_controller.py).
        
        Uses the angle_to_pulse() function from servo_controller.py
        to convert degrees to pulse width.
        
        Args:
            servo_name: Servo name from SERVO_MAP
            angle: Angle in degrees
            controller: ServoController instance for conversion
        
        Example:
            kf.set_servo_angle('left_hip', 30, controller)
        """
        if not SERVO_CONTROLLER_AVAILABLE:
            print("[ERROR] servo_controller.py not available for angle conversion")
            return
        
        config = SERVO_MAP.get(servo_name)
        if config:
            pulse = controller.angle_to_pulse(angle, servo_name)
            self.servos[config.servo_id] = pulse
        else:
            print(f"[ERROR] Servo '{servo_name}' not found in SERVO_MAP")
    
    def set_leg_from_kinematics(self, target_position, is_left: bool, 
                                 leg_ik, controller):
        """
        Set leg servo positions using inverse kinematics (requires kinematics.py).
        
        Uses the kinematics module to calculate joint angles from a target
        foot position in 3D space.
        
        Args:
            target_position: Target foot position (Point3D with x, y, z in cm)
            is_left: True for left leg, False for right leg
            leg_ik: LegIK instance for calculations
            controller: ServoController for angle-to-pulse conversion
        
        Example:
            foot_pos = Point3D(x=0, y=2.8, z=-7.0)
            kf.set_leg_from_kinematics(foot_pos, is_left=True, leg_ik, controller)
        """
        if not KINEMATICS_AVAILABLE:
            print("[ERROR] kinematics.py not available for IK calculations")
            return
        
        angles = leg_ik.calculate_ik(target_position, is_left_leg=is_left)
        if angles:
            prefix = 'left' if is_left else 'right'
            self.set_servo_angle(f'{prefix}_hip', angles.hip_pitch, controller)
            self.set_servo_angle(f'{prefix}_knee', angles.knee, controller)
            self.set_servo_angle(f'{prefix}_ankle', angles.ankle_pitch, controller)
    
    def print_info(self):
        """Print keyframe information."""
        print(f"  Duration: {self.duration_ms}ms")
        print(f"  Servos ({len(self.servos)}):")
        for servo_id, pulse in sorted(self.servos.items()):
            # Find name for this ID
            name = f"[ID:{servo_id}]"
            for n, sid in SERVO_IDS.items():
                if sid == servo_id:
                    name = n
                    break
            print(f"    {name:20s} (ID {servo_id:2d}): {pulse:4d}μs")


# =============================================================================
# ACTION SEQUENCE CLASS
# =============================================================================

class ActionSequence:
    """
    Complete action sequence that can be exported as .d6a file.
    
    Usage:
        action = ActionSequence("MyAction")
        
        kf1 = Keyframe(duration_ms=500)
        kf1.set_servo(13, 1500)  # Left hip
        kf1.set_servo(14, 1500)  # Left knee
        action.add_keyframe(kf1)
        
        action.export_d6a()  # Creates actions/MyAction.d6a
    """
    
    def __init__(self, name: str):
        """
        Create a new action sequence.
        
        Args:
            name: Action name (will be the .d6a filename)
        """
        self.name = name
        self.keyframes: List[Keyframe] = []
    
    def add_keyframe(self, keyframe: Keyframe):
        """Add a keyframe to the sequence."""
        self.keyframes.append(keyframe)
    
    def get_total_duration(self) -> int:
        """Get total duration in milliseconds."""
        return sum(kf.duration_ms for kf in self.keyframes)
    
    def export_d6a(self, output_dir: str = None) -> str:
        """
        Export the action sequence as a .d6a file to the actions folder.
        
        Args:
            output_dir: Directory to save file (default: ../actions/)
        
        Returns:
            Path to the created file
        """
        if output_dir is None:
            # Default to actions folder
            output_dir = os.path.join(os.path.dirname(__file__), '..', 'actions')
        
        # Ensure directory exists
        os.makedirs(output_dir, exist_ok=True)
        
        filepath = os.path.join(output_dir, f"{self.name}.d6a")
        
        # Build binary file content
        data = bytearray()
        
        # =====================================================================
        # D6A FILE FORMAT:
        # Header (64 bytes):
        #   - Action name: 32 bytes (null-padded string)
        #   - Keyframe count: 4 bytes (uint32, little-endian)
        #   - Total duration: 4 bytes (uint32, little-endian)
        #   - Reserved: 24 bytes (zeros)
        #
        # Each Keyframe:
        #   - Duration: 2 bytes (uint16, little-endian)
        #   - Servo count: 2 bytes (uint16, little-endian)
        #   - For each servo:
        #       - Servo ID: 1 byte (uint8)
        #       - Pulse value: 2 bytes (uint16, little-endian)
        #       - Reserved: 1 byte
        # =====================================================================
        
        # Header: action name (32 bytes, null-padded)
        name_bytes = self.name.encode('utf-8')[:31]
        data.extend(name_bytes)
        data.extend(b'\x00' * (32 - len(name_bytes)))
        
        # Header: number of keyframes (4 bytes, little-endian)
        data.extend(struct.pack('<I', len(self.keyframes)))
        
        # Header: total duration (4 bytes)
        data.extend(struct.pack('<I', self.get_total_duration()))
        
        # Header: reserved padding to 64 bytes
        data.extend(b'\x00' * (64 - len(data)))
        
        # Each keyframe
        for kf in self.keyframes:
            # Keyframe duration (2 bytes)
            data.extend(struct.pack('<H', kf.duration_ms))
            
            # Number of servos (2 bytes)
            data.extend(struct.pack('<H', len(kf.servos)))
            
            # Each servo: ID (1 byte) + pulse (2 bytes) + reserved (1 byte)
            for servo_id, pulse in sorted(kf.servos.items()):
                data.extend(struct.pack('<BHB', servo_id, pulse, 0))
        
        # Pad file to standard size (16KB)
        while len(data) < 16384:
            data.extend(b'\x00')
        
        # Write to file
        with open(filepath, 'wb') as f:
            f.write(data)
        
        print(f"\n[D6A GENERATOR] File created successfully!")
        print(f"    Path: {filepath}")
        print(f"    Size: {len(data)} bytes")
        return filepath
    
    def print_summary(self):
        """Print a summary of the action sequence."""
        print(f"\n{'='*60}")
        print(f"ACTION SEQUENCE: {self.name}")
        print(f"{'='*60}")
        print(f"Total Keyframes: {len(self.keyframes)}")
        print(f"Total Duration: {self.get_total_duration()}ms")
        print(f"{'='*60}")
        
        for i, kf in enumerate(self.keyframes):
            print(f"\n[Keyframe {i+1}]")
            kf.print_info()


# =============================================================================
# HELPER FUNCTIONS
# =============================================================================

def get_servo_id(servo_name: str) -> int:
    """
    Get servo ID from servo name.
    
    Args:
        servo_name: Name like 'left_hip', 'right_clamp', etc.
    
    Returns:
        Servo ID (1-18)
    """
    return SERVO_IDS.get(servo_name.lower(), -1)


def print_servo_reference():
    """Print the complete servo reference table."""
    print("\n" + "="*60)
    print("SERVO REFERENCE TABLE")
    print("="*60)
    print(f"{'Name':<20} {'ID':<5} {'Type':<10}")
    print("-"*60)
    
    # Group by type
    print("\n[HEAD - PWM Servos]")
    for name in ['head_tilt', 'head_pan']:
        print(f"  {name:<18} {SERVO_IDS[name]:<5} PWM")
    
    print("\n[LEFT ARM - Bus Servos]")
    for name in ['left_shoulder', 'left_upper_arm', 'left_elbow', 'left_wrist']:
        print(f"  {name:<18} {SERVO_IDS[name]:<5} Bus")
    
    print("\n[RIGHT ARM - Bus Servos]")
    for name in ['right_shoulder', 'right_upper_arm', 'right_elbow', 'right_wrist']:
        print(f"  {name:<18} {SERVO_IDS[name]:<5} Bus")
    
    print("\n[LEFT LEG - Bus Servos]")
    for name in ['left_hip', 'left_knee', 'left_ankle']:
        print(f"  {name:<18} {SERVO_IDS[name]:<5} Bus")
    
    print("\n[RIGHT LEG - Bus Servos]")
    for name in ['right_hip', 'right_knee', 'right_ankle']:
        print(f"  {name:<18} {SERVO_IDS[name]:<5} Bus")
    
    print("\n[CLAMPS - Bus Servos]")
    for name in ['left_clamp', 'right_clamp']:
        print(f"  {name:<18} {SERVO_IDS[name]:<5} Bus")
    
    print("\n" + "-"*60)
    print("Pulse Range: 500-2500μs (center: 1500μs)")
    print("="*60)


# =============================================================================
# EXAMPLE: HOW TO CREATE AN ACTION GROUP
# =============================================================================

def example_create_action():
    """
    Example: Creating an action group step by step.
    
    This demonstrates the workflow:
    1. Create ActionSequence with a name
    2. Create Keyframes with servo positions
    3. Add keyframes to sequence
    4. Export to .d6a file
    """
    print("\n" + "="*60)
    print("EXAMPLE: Creating a Custom Action")
    print("="*60)
    
    # Step 1: Create the action sequence
    action = ActionSequence("ExampleAction")
    
    # -------------------------------------------------------------------------
    # KEYFRAME 1: Initial Standing Position
    # You input: servo ID, pulse value, and duration
    # -------------------------------------------------------------------------
    kf1 = Keyframe(duration_ms=500)  # 500ms to reach this position
    
    # Set servos by ID and pulse value
    # Left leg
    kf1.set_servo(13, 1500)  # Left hip - center
    kf1.set_servo(14, 1500)  # Left knee - center
    kf1.set_servo(15, 1500)  # Left ankle - center
    
    # Right leg
    kf1.set_servo(16, 1500)  # Right hip - center
    kf1.set_servo(3, 1500)   # Right knee - center
    kf1.set_servo(4, 1500)   # Right ankle - center
    
    # Left arm
    kf1.set_servo(5, 1500)   # Left shoulder - center
    kf1.set_servo(7, 1500)   # Left elbow - center
    
    # Right arm
    kf1.set_servo(12, 1500)  # Right shoulder - center
    kf1.set_servo(10, 1500)  # Right elbow - center
    
    # Clamps (open position)
    kf1.set_servo(17, 1500)  # Left clamp - open
    kf1.set_servo(18, 1500)  # Right clamp - open
    
    action.add_keyframe(kf1)
    
    # -------------------------------------------------------------------------
    # KEYFRAME 2: Crouch Position
    # -------------------------------------------------------------------------
    kf2 = Keyframe(duration_ms=600)  # 600ms transition
    
    # OR use servo names instead of IDs:
    kf2.set_servo_by_name('left_hip', 1700)
    kf2.set_servo_by_name('left_knee', 1200)
    kf2.set_servo_by_name('left_ankle', 1300)
    
    kf2.set_servo_by_name('right_hip', 1700)
    kf2.set_servo_by_name('right_knee', 1200)
    kf2.set_servo_by_name('right_ankle', 1300)
    
    # Keep arms neutral
    kf2.set_servo(5, 1500)
    kf2.set_servo(12, 1500)
    
    action.add_keyframe(kf2)
    
    # -------------------------------------------------------------------------
    # KEYFRAME 3: Return to Stand
    # -------------------------------------------------------------------------
    kf3 = Keyframe(duration_ms=500)
    
    # All back to center
    for servo_id in [13, 14, 15, 16, 3, 4, 5, 7, 12, 10]:
        kf3.set_servo(servo_id, 1500)
    
    action.add_keyframe(kf3)
    
    # -------------------------------------------------------------------------
    # Print summary and export
    # -------------------------------------------------------------------------
    action.print_summary()
    
    # Export to actions folder
    filepath = action.export_d6a()
    
    print(f"\n[SUCCESS] Your action file is ready!")
    print(f"[USAGE] In code: AGC.runActionGroup('{action.name}')")
    
    return action


# =============================================================================
# EXAMPLE: USING KINEMATICS TO CREATE ACTION GROUP
# =============================================================================

def example_create_action_with_kinematics():
    """
    To calculate joint angles from target foot positions. 
    We specify WHERE we want the foot (x, y, z coordinates), 
    and the kinematics module uses the Law of Cosines to calculate the required 
    hip, knee, and ankle angles. The servo controller then converts those angles 
    to pulse signals.
    
    This demonstrates the full pipeline:
    1. Define target POSITIONS (x, y, z in cm)
    2. Kinematics calculates joint ANGLES
    3. Servo controller converts angles to PULSES
    4. Export to .d6a file
    
    USE THIS METHOD WHEN:
    - You know WHERE you want the foot to be
    - You don't want to manually calculate angles
    - You need mathematically precise positioning
    """
    
    # Check if kinematics is available
    if not KINEMATICS_AVAILABLE:
        print("[ERROR] kinematics.py not found - cannot run this example")
        return None
    
    if not SERVO_CONTROLLER_AVAILABLE:
        print("[ERROR] servo_controller.py not found - cannot run this example")
        return None
    
    print("\n" + "="*60)
    print("EXAMPLE: Creating Action with KINEMATICS")
    print("="*60)
    
    # Step 1: Initialize kinematics and servo controller
    leg_ik = LegIK(RobotDimensions())
    controller = ServoController()
    
    # Step 2: Create the action sequence
    action = ActionSequence("CrouchWithKinematics")
    
    # -------------------------------------------------------------------------
    # KEYFRAME 1: Standing Position
    # Define foot positions in 3D space (x, y, z in centimeters)
    # -------------------------------------------------------------------------
    kf1 = Keyframe(duration_ms=500)
    
    # Standing: feet directly below hips
    # x = forward/backward, y = left/right, z = up/down (negative = down)
    left_foot_stand = Point3D(x=0, y=2.8, z=-8.0)   # Left foot position
    right_foot_stand = Point3D(x=0, y=-2.8, z=-8.0)  # Right foot position
    
    # Use kinematics to calculate and set servo positions
    kf1.set_leg_from_kinematics(left_foot_stand, is_left=True, 
                                 leg_ik=leg_ik, controller=controller)
    kf1.set_leg_from_kinematics(right_foot_stand, is_left=False,
                                 leg_ik=leg_ik, controller=controller)
    
    # Arms at neutral (using direct pulse for simplicity)
    kf1.set_servo(5, 1500)   # Left shoulder
    kf1.set_servo(12, 1500)  # Right shoulder
    
    action.add_keyframe(kf1)
    
    # -------------------------------------------------------------------------
    # KEYFRAME 2: Crouch Position
    # Move feet forward and up (closer to body)
    # -------------------------------------------------------------------------
    kf2 = Keyframe(duration_ms=600)
    
    # Crouching: feet move forward and up
    left_foot_crouch = Point3D(x=2.0, y=2.8, z=-6.0)   # Forward and up
    right_foot_crouch = Point3D(x=2.0, y=-2.8, z=-6.0)
    
    # Kinematics automatically calculates: hip, knee, ankle angles
    kf2.set_leg_from_kinematics(left_foot_crouch, is_left=True,
                                 leg_ik=leg_ik, controller=controller)
    kf2.set_leg_from_kinematics(right_foot_crouch, is_left=False,
                                 leg_ik=leg_ik, controller=controller)
    
    action.add_keyframe(kf2)
    
    # -------------------------------------------------------------------------
    # KEYFRAME 3: Return to Standing
    # -------------------------------------------------------------------------
    kf3 = Keyframe(duration_ms=500)
    
    # Back to standing position
    kf3.set_leg_from_kinematics(left_foot_stand, is_left=True,
                                 leg_ik=leg_ik, controller=controller)
    kf3.set_leg_from_kinematics(right_foot_stand, is_left=False,
                                 leg_ik=leg_ik, controller=controller)
    
    action.add_keyframe(kf3)
    
    # -------------------------------------------------------------------------
    # Print summary and export
    # -------------------------------------------------------------------------
    action.print_summary()
    
    # Export to actions folder
    filepath = action.export_d6a()
    
    print(f"\n[SUCCESS] Action created using KINEMATICS!")
    print(f"[INFO] Foot positions were converted to joint angles automatically")
    print(f"[USAGE] In code: AGC.runActionGroup('{action.name}')")
    
    return action


# =============================================================================
# MAIN
# =============================================================================

if __name__ == "__main__":
    print("="*60)
    print("TonyPi Action Group Generator")
    print("="*60)
    print("""
This tool generates .d6a action files for the TonyPi robot.

MODULES RELATIONSHIP:
    ┌──────────────────────────────────────────────────────────┐
    │  kinematics.py      servo_controller.py    This File    │
    │       │                    │                   │         │
    │  Calculate angles  →  Convert to pulse  →  Create .d6a  │
    │  from positions       (500-2500μs)        (actions/)    │
    └──────────────────────────────────────────────────────────┘
    """)
    
    # Print servo reference
    print_servo_reference()
    
    # Run example with direct pulse values
    print("\n" + "="*60)
    print("EXAMPLE 1: Direct Pulse Values")
    print("="*60)
    example_create_action()
    
    # Run example with kinematics
    print("\n" + "="*60)
    print("EXAMPLE 2: Using Kinematics")
    print("="*60)
    example_create_action_with_kinematics()

