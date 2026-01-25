#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
TonyPi Bipedal Robot Kinematics Module
=======================================
This module implements inverse kinematics (IK) calculations for the TonyPi 
humanoid robot. It computes joint angles required to position the robot's
end effectors (hands, feet) at desired locations in 3D space.

=== HOW THIS MODULE IS USED FOR ACTION GROUPS ===

1. We define a target position for the foot/hand (x, y, z coordinates in cm)
2. This module calculates the joint angles (hip, knee, ankle) needed
3. Those angles are sent to servo_controller.py to convert to pulse values
4. The pulse values are stored in keyframes to create .d6a action files

EXAMPLE:
    foot_position = Point3D(x=0, y=2.8, z=-7.0)  # Target foot position
    angles = LegIK().calculate_ik(foot_position)  # Get joint angles
    # Result: hip=30°, knee=60°, ankle=-20°

=== KEY MATH CONCEPT ===

We use the LAW OF COSINES to find the knee angle:
    cos(knee) = (thigh² + shin² - distance²) / (2 × thigh × shin)

This is basic triangle math - if we know the leg lengths and how far
the foot needs to be from the hip, we can calculate the knee bend angle.

Key Concepts:
- Forward Kinematics: Given joint angles → Find end effector position
- Inverse Kinematics: Given end effector position → Find joint angles

The robot uses a 16-DOF configuration:
- 2 DOF per arm (shoulder, elbow) x 2 arms = 4 DOF
- 3 DOF per leg (hip, knee, ankle) x 2 legs = 6 DOF  
- 2 DOF head (pan, tilt)
- Additional wrist servos for manipulation

Author: TonyPi FYP Project
"""

import math
from typing import Tuple, Dict, List, Optional
from dataclasses import dataclass


# =============================================================================
# ROBOT PHYSICAL PARAMETERS (measured in centimeters)
# =============================================================================

@dataclass
class RobotDimensions:
    """Physical dimensions of TonyPi robot limbs"""
    # Leg segment lengths (cm)
    hip_offset_y: float = 2.8      # Distance from center to hip joint
    thigh_length: float = 4.5      # Upper leg (hip to knee)
    shin_length: float = 4.5       # Lower leg (knee to ankle)
    foot_height: float = 1.2       # Ankle to ground
    
    # Arm segment lengths (cm)
    shoulder_offset: float = 3.2   # Distance from center to shoulder
    upper_arm: float = 3.8         # Shoulder to elbow
    forearm: float = 4.2           # Elbow to wrist
    hand_length: float = 2.0       # Wrist to fingertip
    
    # Body dimensions
    torso_height: float = 8.5      # Hip to shoulder height
    hip_width: float = 5.6         # Total hip width


# Servo angle limits (in degrees)
SERVO_LIMITS = {
    # Leg servos
    'hip_pitch': (-45, 90),      # Forward/backward rotation
    'hip_roll': (-30, 30),       # Side-to-side rotation
    'knee': (0, 150),            # Knee bend (0 = straight)
    'ankle_pitch': (-45, 45),    # Foot up/down
    'ankle_roll': (-30, 30),     # Foot tilt
    
    # Arm servos
    'shoulder_pitch': (-90, 180), # Arm forward/backward
    'shoulder_roll': (-30, 90),   # Arm out to side
    'elbow': (0, 135),            # Elbow bend
    'wrist': (-90, 90),           # Wrist rotation
}


# =============================================================================
# COORDINATE SYSTEMS
# =============================================================================
"""
Robot Coordinate Frame (Right-hand rule):
    
         +Z (up)
          |
          |
          |_______ +Y (left)
         /
        /
       +X (forward)
    
Origin is at the center of the hip joint line.
"""


@dataclass
class Point3D:
    """3D point in robot coordinate frame"""
    x: float  # Forward/backward
    y: float  # Left/right
    z: float  # Up/down
    
    def distance_to(self, other: 'Point3D') -> float:
        """Euclidean distance to another point"""
        return math.sqrt(
            (self.x - other.x)**2 + 
            (self.y - other.y)**2 + 
            (self.z - other.z)**2
        )
    
    def __add__(self, other: 'Point3D') -> 'Point3D':
        return Point3D(self.x + other.x, self.y + other.y, self.z + other.z)


@dataclass  
class JointAngles:
    """Joint angles for a single leg (in degrees)"""
    hip_pitch: float = 0.0    # Forward/backward swing
    hip_roll: float = 0.0     # Side-to-side tilt
    knee: float = 0.0         # Knee bend angle
    ankle_pitch: float = 0.0  # Foot up/down angle
    ankle_roll: float = 0.0   # Foot side tilt
    
    def as_dict(self) -> Dict[str, float]:
        return {
            'hip_pitch': self.hip_pitch,
            'hip_roll': self.hip_roll,
            'knee': self.knee,
            'ankle_pitch': self.ankle_pitch,
            'ankle_roll': self.ankle_roll
        }


# =============================================================================
# INVERSE KINEMATICS ENGINE
# =============================================================================

class LegIK:
    """
    Inverse Kinematics solver for TonyPi leg.
    
    === SIMPLE EXPLANATION ===
    
    Given: "I want the foot at position (x, y, z)"
    Output: "You need hip=30°, knee=60°, ankle=-20°"
    
    === HOW IT WORKS ===
    
    Step 1: Calculate distance from hip to target foot position
            distance = sqrt(x² + z²)
    
    Step 2: Use Law of Cosines to find knee angle
            cos(knee) = (L1² + L2² - D²) / (2 × L1 × L2)
            where L1=thigh length, L2=shin length, D=distance
    
    Step 3: Use atan2 to find hip angle (forward/backward tilt)
    
    Step 4: Calculate ankle angle to keep foot level on ground
    
    This is used by action_group_generator.py to calculate
    servo positions for creating walking and crouching actions.
    """
    
    def __init__(self, dimensions: RobotDimensions = None):
        self.dim = dimensions or RobotDimensions()
        
    def calculate_ik(self, 
                     foot_position: Point3D, 
                     is_left_leg: bool = True) -> Optional[JointAngles]:
        """
        Calculate joint angles to place foot at desired position.
        
        Args:
            foot_position: Target foot position in hip-centered coordinates
            is_left_leg: True for left leg, False for right leg
            
        Returns:
            JointAngles object with calculated angles, or None if unreachable
            
        Algorithm:
            1. Transform foot position to leg-local coordinates
            2. Solve 2D IK in sagittal plane (side view) for hip_pitch and knee
            3. Solve for hip_roll based on lateral offset
            4. Calculate ankle angles to keep foot level
        """
        
        # Step 1: Transform to leg-local coordinates
        # Offset for left or right leg
        hip_y_offset = self.dim.hip_offset_y if is_left_leg else -self.dim.hip_offset_y
        
        # Local foot position relative to hip joint
        local_x = foot_position.x
        local_y = foot_position.y - hip_y_offset
        local_z = foot_position.z + self.dim.thigh_length + self.dim.shin_length  # Relative to hip
        
        # Step 2: Calculate leg length (hip to foot distance in sagittal plane)
        L1 = self.dim.thigh_length  # Thigh length
        L2 = self.dim.shin_length   # Shin length
        
        # Distance from hip to foot in the XZ plane
        D_xz = math.sqrt(local_x**2 + local_z**2)
        
        # Total distance including Y offset (for hip roll)
        D_total = math.sqrt(D_xz**2 + local_y**2)
        
        # Step 3: Check reachability
        # Foot must be within reach: |L1 - L2| < D < L1 + L2
        min_reach = abs(L1 - L2) + 0.5  # Add small margin
        max_reach = L1 + L2 - 0.5       # Subtract small margin
        
        if D_xz < min_reach or D_xz > max_reach:
            print(f"[IK] Target unreachable: distance={D_xz:.2f}, range=[{min_reach:.2f}, {max_reach:.2f}]")
            return None
        
        # Step 4: Solve knee angle using Law of Cosines
        # c² = a² + b² - 2ab·cos(C)
        # Rearranged: cos(C) = (a² + b² - c²) / (2ab)
        
        cos_knee = (L1**2 + L2**2 - D_xz**2) / (2 * L1 * L2)
        cos_knee = max(-1, min(1, cos_knee))  # Clamp to valid range
        
        knee_angle = math.degrees(math.acos(cos_knee))
        
        # Step 5: Solve hip pitch angle
        # Using atan2 for quadrant-aware angle calculation
        
        # Angle from vertical to the line connecting hip to foot
        alpha = math.atan2(local_x, -local_z)
        
        # Angle offset due to knee bend (using law of sines)
        # sin(beta)/L2 = sin(knee)/D_xz
        sin_beta = L2 * math.sin(math.radians(knee_angle)) / D_xz
        sin_beta = max(-1, min(1, sin_beta))
        beta = math.asin(sin_beta)
        
        hip_pitch = math.degrees(alpha + beta)
        
        # Step 6: Solve hip roll (lateral leg swing)
        hip_roll = math.degrees(math.atan2(local_y, D_xz))
        
        # Step 7: Calculate ankle angles to keep foot level
        # Ankle pitch compensates for hip and knee angles
        ankle_pitch = -(hip_pitch + (180 - knee_angle))
        
        # Ankle roll compensates for hip roll
        ankle_roll = -hip_roll
        
        # Step 8: Clamp angles to servo limits
        angles = JointAngles(
            hip_pitch=self._clamp_angle(hip_pitch, 'hip_pitch'),
            hip_roll=self._clamp_angle(hip_roll, 'hip_roll'),
            knee=self._clamp_angle(knee_angle, 'knee'),
            ankle_pitch=self._clamp_angle(ankle_pitch, 'ankle_pitch'),
            ankle_roll=self._clamp_angle(ankle_roll, 'ankle_roll')
        )
        
        return angles
    
    def _clamp_angle(self, angle: float, joint_name: str) -> float:
        """Clamp angle to valid servo range"""
        limits = SERVO_LIMITS.get(joint_name, (-180, 180))
        return max(limits[0], min(limits[1], angle))
    
    def calculate_foot_trajectory(self, 
                                   start: Point3D, 
                                   end: Point3D,
                                   step_height: float = 2.0,
                                   num_points: int = 20) -> List[Point3D]:
        """
        Generate a smooth foot trajectory for walking.
        
        Uses a cycloid-like curve to lift the foot, preventing dragging
        and providing smooth ground contact.
        
        Args:
            start: Starting foot position
            end: Target foot position  
            step_height: Maximum height of foot lift (cm)
            num_points: Number of trajectory points
            
        Returns:
            List of Point3D positions along the trajectory
        """
        trajectory = []
        
        for i in range(num_points):
            t = i / (num_points - 1)  # Parameter from 0 to 1
            
            # Linear interpolation for X and Y
            x = start.x + t * (end.x - start.x)
            y = start.y + t * (end.y - start.y)
            
            # Cycloid curve for Z (height) - smoother than sine
            # z = h * (1 - cos(π * t)) / 2
            z_lift = step_height * (1 - math.cos(math.pi * t)) / 2
            z = start.z + t * (end.z - start.z) + z_lift
            
            trajectory.append(Point3D(x, y, z))
        
        return trajectory


class ArmIK:
    """
    Inverse Kinematics solver for TonyPi arm.
    
    Simplified 2-DOF arm model (shoulder pitch + elbow).
    Used for pick-and-place operations.
    """
    
    def __init__(self, dimensions: RobotDimensions = None):
        self.dim = dimensions or RobotDimensions()
    
    def calculate_ik(self, 
                     hand_position: Point3D,
                     is_left_arm: bool = True) -> Optional[Dict[str, float]]:
        """
        Calculate arm joint angles to reach target position.
        
        Args:
            hand_position: Target hand position
            is_left_arm: True for left arm
            
        Returns:
            Dictionary with shoulder and elbow angles
        """
        L1 = self.dim.upper_arm
        L2 = self.dim.forearm
        
        # Calculate distance to target (in arm plane)
        dx = hand_position.x
        dz = hand_position.z - self.dim.torso_height  # Relative to shoulder
        
        D = math.sqrt(dx**2 + dz**2)
        
        # Check reachability
        if D > L1 + L2 or D < abs(L1 - L2):
            return None
        
        # Elbow angle (law of cosines)
        cos_elbow = (L1**2 + L2**2 - D**2) / (2 * L1 * L2)
        cos_elbow = max(-1, min(1, cos_elbow))
        elbow_angle = math.degrees(math.acos(cos_elbow))
        
        # Shoulder angle
        alpha = math.atan2(dx, dz)
        beta = math.acos((L1**2 + D**2 - L2**2) / (2 * L1 * D))
        shoulder_angle = math.degrees(alpha + beta)
        
        return {
            'shoulder_pitch': shoulder_angle,
            'elbow': 180 - elbow_angle  # Convert to servo convention
        }


# =============================================================================
# FORWARD KINEMATICS (for verification)
# =============================================================================

class ForwardKinematics:
    """
    Forward kinematics calculator.
    Given joint angles, compute end effector position.
    Used to verify IK solutions.
    """
    
    def __init__(self, dimensions: RobotDimensions = None):
        self.dim = dimensions or RobotDimensions()
    
    def calculate_foot_position(self, 
                                 angles: JointAngles,
                                 is_left_leg: bool = True) -> Point3D:
        """
        Calculate foot position from joint angles.
        
        Uses Denavit-Hartenberg transformation matrices.
        """
        L1 = self.dim.thigh_length
        L2 = self.dim.shin_length
        
        # Convert to radians
        hip_rad = math.radians(angles.hip_pitch)
        knee_rad = math.radians(angles.knee)
        
        # Position calculation using rotation matrices
        # Simplified 2D case (sagittal plane)
        
        # Knee position relative to hip
        knee_x = L1 * math.sin(hip_rad)
        knee_z = -L1 * math.cos(hip_rad)
        
        # Foot position relative to knee
        total_angle = hip_rad + math.radians(180 - angles.knee)
        foot_x = knee_x + L2 * math.sin(total_angle)
        foot_z = knee_z - L2 * math.cos(total_angle)
        
        # Add hip offset for left/right leg
        hip_y = self.dim.hip_offset_y if is_left_leg else -self.dim.hip_offset_y
        foot_y = hip_y + L1 * math.sin(math.radians(angles.hip_roll))
        
        return Point3D(foot_x, foot_y, foot_z)


# =============================================================================
# USAGE EXAMPLE
# =============================================================================

if __name__ == "__main__":
    # Initialize IK solver
    leg_ik = LegIK()
    
    # Define target foot position (standing position)
    target = Point3D(x=0, y=2.8, z=-8.0)  # Foot directly below hip
    
    print("=== TonyPi Inverse Kinematics Demo ===")
    print(f"Target foot position: ({target.x}, {target.y}, {target.z}) cm")
    
    # Calculate joint angles
    angles = leg_ik.calculate_ik(target, is_left_leg=True)
    
    if angles:
        print("\nCalculated joint angles (degrees):")
        for joint, angle in angles.as_dict().items():
            print(f"  {joint}: {angle:.2f}°")
        
        # Verify with forward kinematics
        fk = ForwardKinematics()
        result = fk.calculate_foot_position(angles, is_left_leg=True)
        print(f"\nFK verification: ({result.x:.2f}, {result.y:.2f}, {result.z:.2f}) cm")
    else:
        print("Target position is unreachable!")
