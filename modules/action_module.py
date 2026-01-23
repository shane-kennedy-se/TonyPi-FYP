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

    def run_transport_cardboard(self, camera=None, get_frame=None):
        """Execute transport cardboard sequence with QR code navigation
        
        Uses the qr_navigate module for robust QR scanning and navigation.
        
        Args:
            camera: The camera object from main.py (deprecated, use get_frame instead).
            get_frame: A callable that returns (ret, frame) for getting latest frame.
                      This is the preferred method to avoid camera threading conflicts.
        """
        print("=== TonyPi Pro: Transport Cardboard Sequence ===")
        time.sleep(1)

        try:
            # Import qr_navigate module
            from modules import qr_navigate
            
            # Determine frame source - prefer get_frame function over raw camera
            if get_frame is not None:
                frame_source = get_frame
            elif camera is not None:
                frame_source = camera.read
            else:
                print("[ERROR] No camera or get_frame provided. Cannot scan for QR.")
                return False

            # Pick up the object
            print("Picking up cardboard...")
            AGC.runActionGroup("PickUpDiecut2")
            time.sleep(1)

            # Use qr_navigate module to scan and navigate to destination
            print("Scanning for destination QR code...")
            
            # Update the shared frame for qr_navigate
            def update_navigation_frame():
                """Continuously update the shared frame for navigation"""
                ret, frame = frame_source()
                if ret and frame is not None:
                    qr_navigate.current_frame_shared = frame.copy()
            
            # Start QR navigation
            success = qr_navigate.start_qr_navigation_async(timeout=60)
            if not success:
                print("[ERROR] Navigation already in progress")
                return False
            
            # Keep feeding frames to qr_navigate until navigation completes
            print("Navigating to destination station...")
            while qr_navigate.navigation_active:
                update_navigation_frame()
                time.sleep(0.02)
            
            # Get the result
            destination = qr_navigate.get_navigation_result()
            
            if destination:
                print(f"✅ Arrived at destination: {destination}")
            else:
                print("[WARNING] Navigation failed or timed out")
                return False
            
            # Put down the object
            print("Placing cardboard down...")
            AGC.runActionGroup("PutDown1")
            time.sleep(1)

            print("=== Task Complete ===")
            return True

        except Exception as e:
            print(f"[ERROR] Transport Cardboard failed: {e}")
            import traceback
            traceback.print_exc()
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
