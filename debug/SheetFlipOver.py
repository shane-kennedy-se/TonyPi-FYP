#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import time
import os

# Ensure Python 3 is being used
if sys.version_info.major < 3:
    print('Please run this program with Python 3!')
    sys.exit(0)

# Import TonyPi Action Group Control module
if __name__ == '__main__':
    sys.path.append('/home/pi/tonypi/functions')  # Path when running directly
else:
    sys.path.append(os.path.join(os.path.dirname(__file__), 'functions'))  # Path when imported

import hiwonder.ActionGroupControl as AGC
from hiwonder import Controller
from hiwonder import ros_robot_controller_sdk as rrc

# ---------------------------
# TonyPi setup
# ---------------------------
rrc_board = rrc.Board()
Board = Controller.Controller(rrc_board)

# ---------------------------
# Main Task
# ---------------------------
def main():
    print("=== TonyPi Pro: Sheet Flip Sequence ===")
    time.sleep(2)

    try:
        # Step 1: Grab sheet
        print("Step 1: Grabbing sheet...")
        Board.set_bus_servo_deviation(16, 57)
        Board.set_bus_servo_deviation(18, 0)
        AGC.runActionGroup("GrabSheet")
        time.sleep(1)

        # Step 2: Pass sheet
        print("Step 2: Passing sheet...")
        Board.set_bus_servo_deviation(16, 57)
        Board.set_bus_servo_deviation(18, 0)
        AGC.runActionGroup("PassSheet")
        time.sleep(1)

        # Step 3: Change hand (flip sheet)
        print("Step 3: Changing hand (flip sheet)...")
        Board.set_bus_servo_deviation(16, -34)
        Board.set_bus_servo_deviation(18, -125)
        AGC.runActionGroup("ChangeHandSheet")
        time.sleep(1)

        # Step 4: Put down sheet
        print("Step 4: Putting down sheet...")
        Board.set_bus_servo_deviation(18, 0)
        AGC.runActionGroup("PutDownSheet")
        Board.set_bus_servo_deviation(16, 0)
        time.sleep(1)

        print("=== Task Complete ===")

    except KeyboardInterrupt:
        print("\n[INFO] Stopped by user")

if __name__ == '__main__':
    main()
