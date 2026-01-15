import cv2
import hiwonder.ActionGroupControl as AGC
from hiwonder import Controller, ros_robot_controller_sdk as rrc
import time
from pyzbar import pyzbar

rrc_board = rrc.Board()
Board = Controller.Controller(rrc_board)

# Servo IDs
HEAD_PAN_SERVO = 2
HEAD_TILT_SERVO = 1

qr_scanning = False

import threading

def run_action_async(action_name):
    t = threading.Thread(
        target=AGC.runActionGroup,
        args=(action_name,),
        daemon=True
    )
    t.start()

# ---------------- HEAD CONTROL ----------------
def rotate_head(direction='left'):
    try:
        if direction == 'left':
            pan = 1200
            tilt = 800
            print("👀 LEFT")

        elif direction == 'right':
            pan = 400
            tilt = 800
            print("👀 RIGHT")

        elif direction == 'up':
            pan = 800
            tilt = 500
            print("👀 UP")

        elif direction == 'down':
            pan = 800
            tilt = 1100
            print("👀 DOWN")

        else:
            pan = 800
            tilt = 800
            print("👀 CENTER")

        rrc_board.pwm_servo_set_position(
            0.3,
            [
                [HEAD_PAN_SERVO, pan],
                [HEAD_TILT_SERVO, tilt]
            ]
        )
        time.sleep(0.3)

    except Exception as e:
        print(f"[WARNING] Head servo failed: {e}")

# ---------------- QR NAVIGATION ----------------
def navigate_to_station(frame_getter, timeout=60):

    global qr_scanning

    print("[INFO] Starting QR scan...")
    qr_scanning = True
    frames_without_qr = 0
    station_detected = None
    start_time = time.time()

    # Scan pattern
    scan_pattern = ['left', 'right', 'up', 'down']
    scan_index = 0

    while qr_scanning:

        # Timeout
        if time.time() - start_time > timeout:
            print("[INFO] Timeout")
            break

        frame = frame_getter()
        if frame is None:
            time.sleep(0.05)
            continue

        barcodes = pyzbar.decode(frame)

        # -------- QR FOUND --------
        if barcodes:
            for barcode in barcodes:

                data = barcode.data.decode("utf-8")
                print(f"✓ QR Detected: {data}")

                rotate_head('center')

                (x, y, w, h) = barcode.rect
                x_center = x + w // 2
                qr_width = w
                frame_center = frame.shape[1] // 2

                # ------ ALIGN BODY ------
                if x_center < frame_center - 60:
                    print("↩ Turning LEFT")
                    run_action_async('turn_left')

                elif x_center > frame_center + 60:
                    print("↪ Turning RIGHT")
                    run_action_async('turn_right')

                # ------ MOVE FORWARD ------
                else:
                    if qr_width < 120:
                        print("→ Moving forward")
                        run_action_async('go_foward')
                        time.sleep(0.2)
                    else:
                        print(f"🎯 STATION REACHED: {data}")
                        station_detected = data
                        qr_scanning = False
                        break

        # -------- SEARCH MODE --------
        else:
            frames_without_qr += 1

            if frames_without_qr % 15 == 0:

                direction = scan_pattern[scan_index]
                print(f"🔍 Searching {direction}")

                rotate_head(direction)

                # Next direction
                scan_index = (scan_index + 1) % len(scan_pattern)

        time.sleep(0.05)

    # Reset head
    print("🔄 Resetting head to CENTER")
    rotate_head('center')

    print(f"[INFO] Scan complete → {station_detected}")
    return station_detected

if __name__ == "__main__":
    print("Use navigate_to_station(frame_getter) from main.py")
