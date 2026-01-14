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
    from gpiozero import Button
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
    
    class MockButton:
        def __init__(self, pin, pull_up=True):
            pass
        def when_pressed(self, callback):
            pass
    
    AGC = MockAGC()
    Controller = MockController
    rrc = type('MockRRC', (), {'Board': MockBoard})()
    Button = MockButton


class RobotActions:
    """Handles all robot action sequences"""
    
    def __init__(self):
    # Hardware setup - only if available
        if HARDWARE_AVAILABLE:
            self.rrc_board = rrc.Board()
            self.Board = Controller.Controller(self.rrc_board)

            # FIX HERE
            self.agc = AGC   # <-- use module directly

            # Touch sensor setup
            TOUCH_PIN = 18
            self.touch = Button(TOUCH_PIN, pull_up=True)
            self.emergency_stop = False
        
            def on_touch_detected():
                self.emergency_stop = True
                print("\n⚠ TOUCH DETECTED! EMERGENCY STOP ACTIVATED ⚠")
                self.agc.stopAction()   # or stopAll if exists
        
            self.touch.when_pressed = on_touch_detected
        else:
            self.rrc_board = None
            self.Board = MockController(None)
            self.agc = MockAGC()
            self.touch = None
            self.emergency_stop = False

    
    def run_diecut_peeling(self):
        """Execute diecut peeling sequence"""
        print("=== TonyPi Pro: Diecut Peeling Sequence ===")
        time.sleep(2)
        
        try:
            # Step 1: Pick up the diecut
            if self.emergency_stop: return False
            print("Step 1: Picking up diecut...")
            self.Board.set_bus_servo_deviation(17, 0)
            AGC.runActionGroup("PickUpDiecut1")
            time.sleep(0.5)

            # Step 2: Perform diecut peeling
            if self.emergency_stop: return False
            print("Step 2a: Performing diecut peeling (left hand)...")
            self.Board.set_bus_servo_deviation(17, -50)
            AGC.runActionGroup("DoDiecutLeftHand")
            time.sleep(0.5)

            if self.emergency_stop: return False
            print("Step 2b: Performing diecut peeling (right hand)...")
            self.Board.set_bus_servo_deviation(17, 0)
            AGC.runActionGroup("DoDiecutRightHand")
            time.sleep(0.5)

            # Step 3: Put down the diecut
            if self.emergency_stop: return False
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
            if self.emergency_stop: return False
            print("Step 1: Grabbing label...")
            AGC.runActionGroup("GrabLabel")
            time.sleep(STEP_DELAY)

            # Step 2: Lift label for insertion
            if self.emergency_stop: return False
            print("Step 2: Lifting label for insertion...")
            AGC.runActionGroup("LiftLabelInsertion")
            time.sleep(STEP_DELAY)

            # Step 3: Put label
            if self.emergency_stop: return False
            print("Step 3: Placing label...")
            AGC.runActionGroup("PutLabel")
            time.sleep(STEP_DELAY)

            # Step 4: Put down label and stand up
            if self.emergency_stop: return False
            print("Step 4: Putting down label and standing up...")
            AGC.runActionGroup("PutDownLabelAndStandUp")
            time.sleep(STEP_DELAY)

            print("=== Task Complete ===")
            return True

        except Exception as e:
            print(f"[ERROR] Label Insertion failed: {e}")
            return False

    def run_transport_box(self, steps=5):
        """Execute transport box sequence
        
        Args:
            steps (int): Number of steps to walk forward
        """
        print(f"=== TonyPi Pro: Transport Box Sequence ({steps} steps) ===")
        time.sleep(2)

        try:
            # Step 1: Pick up the object
            if self.emergency_stop: return False
            print("Step 1: Picking up object...")
            AGC.runActionGroup("PickUp")
            time.sleep(0.5)

            # Step 2: Walk forward
            if self.emergency_stop: return False
            print("Step 2: Walking forward...")
            for i in range(steps):
                if self.emergency_stop: return False
                print(f"  Walking step {i+1}/{steps}...")
                AGC.runActionGroup("WalkOneStep")
                time.sleep(0.5)

            # Step 3: Put down the object
            if self.emergency_stop: return False
            print("Step 3: Placing object down...")
            AGC.runActionGroup("PutDown")

            print("=== Task Complete ===")
            return True

        except Exception as e:
            print(f"[ERROR] Transport Box failed: {e}")
            return False

    def run_sheet_flip_over(self):
        """Execute sheet flip over sequence"""
        print("=== TonyPi Pro: Sheet Flip Sequence ===")
        time.sleep(2)

        try:
            # Step 1: Grab sheet
            if self.emergency_stop: return False
            print("Step 1: Grabbing sheet...")
            self.Board.set_bus_servo_deviation(16, 57)
            self.Board.set_bus_servo_deviation(18, 0)
            self.agc.runActionGroup("GrabSheet")
            time.sleep(1)

            # Step 2: Pass sheet
            if self.emergency_stop: return False
            print("Step 2: Passing sheet...")
            self.Board.set_bus_servo_deviation(16, 57)
            self.Board.set_bus_servo_deviation(18, 0)
            self.agc.runActionGroup("PassSheet")
            time.sleep(1)

            # Step 3: Change hand (flip sheet)
            if self.emergency_stop: return False
            print("Step 3: Changing hand (flip sheet)...")
            self.Board.set_bus_servo_deviation(16, -34)
            self.Board.set_bus_servo_deviation(18, -125)
            self.agc.runActionGroup("ChangeHandSheet")
            time.sleep(1)

            # Step 4: Put down sheet
            if self.emergency_stop: return False
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