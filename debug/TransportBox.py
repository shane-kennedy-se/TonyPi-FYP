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


def main():
    # Require steps as a command-line argument
    if len(sys.argv) < 2:
        print("Usage: python3 TransportBox.py <steps>")
        sys.exit(1)

    try:
        steps_to_take = int(sys.argv[1])
        if steps_to_take <= 0:
            print("[ERROR] Steps must be a positive number!")
            sys.exit(1)
    except ValueError:
        print("[ERROR] Steps must be an integer!")
        sys.exit(1)

    print(f"=== TonyPi Pro: Transport Box Sequence ({steps_to_take} steps) ===")
    time.sleep(2)

    try:
        # Step 1: Pick up the object
        print("Step 1: Picking up object...")
        AGC.runActionGroup("PickUp")
        time.sleep(0.5)

        # Step 2: Walk forward
        print("Step 2: Walking forward...")
        for i in range(steps_to_take):
            print(f"  Walking step {i+1}/{steps_to_take}...")
            AGC.runActionGroup("WalkOneStep")
            time.sleep(0.5)

        # Step 3: Put down the object
        print("Step 3: Placing object down...")
        AGC.runActionGroup("PutDown")

        print("=== Task Complete ===")

    except KeyboardInterrupt:
        print("\n[INFO] Stopped by user")


if __name__ == '__main__':
    main()
