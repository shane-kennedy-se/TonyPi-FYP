#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
TonyPi Actions Module
Contains all robot action sequences for automated tasks.
"""

import time
import sys

# Handle missing hardware modules gracefully for development
try:
    import hiwonder.ActionGroupControl as AGC
    import hiwonder.Controller as Controller
    import hiwonder.ros_robot_controller_sdk as rrc
    HARDWARE_AVAILABLE = True
except ImportError:
    print("[WARNING] Hardware modules not available. Running in simulation mode.")
    HARDWARE_AVAILABLE = False
    
    # Mock classes for development
    class MockAGC:
        @staticmethod
        def runActionGroup(action):
            print(f"[SIMULATION] Running action group: {action}")
    
    class MockController:
        def __init__(self, board):
            pass
        def set_bus_servo_deviation(self, servo, deviation):
            print(f"[SIMULATION] Setting servo {servo} to deviation {deviation}")
    
    class MockBoard:
        pass
    
    AGC = MockAGC()
    Controller = MockController
    rrc = type('MockRRC', (), {'Board': MockBoard})()


class RobotActions:
    """Handles all robot action sequences"""
    
    def __init__(self):
    # Hardware setup - only if available
        if HARDWARE_AVAILABLE:
            self.rrc_board = rrc.Board()
            self.Board = Controller.Controller(self.rrc_board)
            self.agc = AGC
        else:
            self.rrc_board = None
            self.Board = MockController(None)
            self.agc = MockAGC()
    
    def run_diecut_peeling(self):
        """Execute diecut peeling sequence"""
        print("=== TonyPi Pro: Diecut Peeling Sequence ===")
        time.sleep(2)
        
        try:
            # Step 1: Pick up the diecut
            print("Step 1: Picking up diecut...")
            self.Board.set_bus_servo_deviation(17, 0)
            AGC.runActionGroup("PickUpDiecut1")
            time.sleep(0.5)

            # Step 2: Perform diecut peeling
            print("Step 2a: Performing diecut peeling (left hand)...")
            self.Board.set_bus_servo_deviation(17, -50)
            AGC.runActionGroup("DoDiecutLeftHand")
            time.sleep(0.5)

            print("Step 2b: Performing diecut peeling (right hand)...")
            self.Board.set_bus_servo_deviation(17, 0)
            AGC.runActionGroup("DoDiecutRightHand")
            time.sleep(0.5)

            # Step 3: Put down the diecut
            print("Step 3: Placing diecut down...")
            self.Board.set_bus_servo_deviation(17, 0)
            AGC.runActionGroup("PutDownDiecut1")
            time.sleep(0.5)

            print("=== Task Complete ===")
            return True

        except Exception as e:
            print(f"[ERROR] Diecut Peeling failed: {e}")
            return False

    def run_label_insertion(self):
        """Execute label insertion sequence"""
        STEP_DELAY = 2
        print("=== TonyPi Pro: Label Insertion Sequence ===")
        time.sleep(2)

        try:
            # Step 1: Grab the label
            print("Step 1: Grabbing label...")
            AGC.runActionGroup("GrabSheetNew")
            time.sleep(STEP_DELAY)

            # Step 2: Lift label for insertion
            print("Step 2: Lifting label for insertion...")
            AGC.runActionGroup("LiftLabelInsertion1")
            time.sleep(STEP_DELAY)

            # Step 3: Put label
            print("Step 3: Placing label...")
            AGC.runActionGroup("PutLabel")
            time.sleep(STEP_DELAY)

            # Step 4: Put down label and stand up
            print("Step 4: Putting down label and standing up...")
            AGC.runActionGroup("PutDownLabelAndStandUp")
            time.sleep(STEP_DELAY)

            print("=== Task Complete ===")
            return True

        except Exception as e:
            print(f"[ERROR] Label Insertion failed: {e}")
            return False

    def run_transport_cardboard(self):
        """Execute transport cardboard sequence with QR code navigation"""
        print("=== TonyPi Pro: Transport Cardboard Sequence ===")
        time.sleep(2)

        try:
            # Import QR scanner for station detection
            try:
                import cv2
                from pyzbar import pyzbar
                from hiwonder import Controller, ros_robot_controller_sdk as rrc
            except ImportError as e:
                print(f"[ERROR] Required modules not available: {e}")
                return False

            # Setup camera and head servo control
            rrc_board = rrc.Board()
            ctl = Controller.Controller(rrc_board)
            cap = cv2.VideoCapture(0)
            
            HEAD_PAN_SERVO = 2
            HEAD_TILT_SERVO = 1
            PAN_CENTER = 1450
            TILT_CENTER = 1150
            
            # Center head for scanning
            ctl.set_pwm_servo_pulse(HEAD_TILT_SERVO, TILT_CENTER, 100)
            ctl.set_pwm_servo_pulse(HEAD_PAN_SERVO, PAN_CENTER, 100)
            time.sleep(0.5)

            # Pick up the object
            print("Picking up object...")
            AGC.runActionGroup("PickUpDiecut2")
            time.sleep(0.5)

            # Scan for QR code to determine destination
            print("Scanning for destination QR code...")
            destination = None
            scan_timeout = time.time() + 30  # 30 second timeout for initial scan
            
            # Head scanning variables
            x_dis = PAN_CENTER
            d_x = 20
            SERVO_PAN_MIN = 1000
            SERVO_PAN_MAX = 1900
            
            while destination is None and time.time() < scan_timeout:
                ret, frame = cap.read()
                if not ret:
                    continue
                
                frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                barcodes = pyzbar.decode(frame_rgb)
                
                if barcodes:
                    barcode = barcodes[0]
                    destination = barcode.data.decode("utf-8")
                    print(f"Destination detected: {destination}")
                    break
                else:
                    # Scan head left-right to find QR
                    x_dis += d_x
                    if x_dis >= SERVO_PAN_MAX or x_dis <= SERVO_PAN_MIN:
                        d_x = -d_x
                    ctl.set_pwm_servo_pulse(HEAD_PAN_SERVO, x_dis, 20)
                
                time.sleep(0.02)
            
            if destination is None:
                print("[WARNING] No QR code found, cannot determine destination")
                cap.release()
                return False
            
            # Center head for navigation
            ctl.set_pwm_servo_pulse(HEAD_PAN_SERVO, PAN_CENTER, 100)
            time.sleep(0.3)
            
            # Navigate to destination by tracking QR and walking
            print(f"Navigating to destination: {destination}")
            nav_timeout = time.time() + 60
            TARGET_WIDTH = 145  # Width threshold indicating arrival
            
            while time.time() < nav_timeout:
                ret, frame = cap.read()
                if not ret:
                    continue
                
                frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                barcodes = pyzbar.decode(frame_rgb)
                
                if barcodes:
                    barcode = barcodes[0]
                    (x, y, w, h) = barcode.rect
                    center_x = x + (w // 2)
                    qr_width = w
                    
                    # Check if we've arrived (QR is large enough)
                    if qr_width >= TARGET_WIDTH:
                        print(f"Arrived at destination: {destination}")
                        break
                    
                    # Align with QR position
                    if center_x < 240:
                        print("Turning left...")
                        AGC.runActionGroup("turn_left")
                    elif center_x > 400:
                        print("Turning right...")
                        AGC.runActionGroup("turn_right")
                    else:
                        # Walk forward using WalkOneStep1
                        print("Walking forward...")
                        AGC.runActionGroup("WalkOneStep1")
                    
                    time.sleep(0.3)
                else:
                    # QR lost, try scanning
                    x_dis += d_x
                    if x_dis >= SERVO_PAN_MAX or x_dis <= SERVO_PAN_MIN:
                        d_x = -d_x
                    ctl.set_pwm_servo_pulse(HEAD_PAN_SERVO, x_dis, 20)
                    time.sleep(0.05)
            
            cap.release()
            
            # Put down the object
            print("Placing object down...")
            AGC.runActionGroup("PutDown1")

            print("=== Task Complete ===")
            return True

        except Exception as e:
            print(f"[ERROR] Transport Cardboard failed: {e}")
            return False

    def run_sheet_flip_over(self):
        """Execute sheet flip over sequence"""
        print("=== TonyPi Pro: Sheet Flip Sequence ===")
        time.sleep(2)

        try:
            # Step 1: Grab sheet
            print("Step 1: Grabbing sheet...")
            self.Board.set_bus_servo_deviation(16, 57)
            self.Board.set_bus_servo_deviation(18, 0)
            self.agc.runActionGroup("GrabSheet")
            time.sleep(1)

            # Step 2: Pass sheet
            print("Step 2: Passing sheet...")
            self.Board.set_bus_servo_deviation(16, 57)
            self.Board.set_bus_servo_deviation(18, 0)
            self.agc.runActionGroup("PassSheet")
            time.sleep(1)

            # Step 3: Change hand (flip sheet)
            print("Step 3: Changing hand (flip sheet)...")
            self.Board.set_bus_servo_deviation(16, -34)
            self.Board.set_bus_servo_deviation(18, -125)
            self.agc.runActionGroup("ChangeHandSheet")
            time.sleep(1)

            # Step 4: Put down sheet
            print("Step 4: Putting down sheet...")
            self.Board.set_bus_servo_deviation(18, 0)
            self.agc.runActionGroup("PutDownSheet")
            self.Board.set_bus_servo_deviation(16, 0)
            time.sleep(1)

            print("=== Task Complete ===")
            return True

        except Exception as e:
            print(f"[ERROR] Sheet Flip Over failed: {e}")
            return False
