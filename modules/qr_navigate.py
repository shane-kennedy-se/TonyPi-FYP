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
import queue

# Thread-safe communication
scan_result_queue = queue.Queue()
current_frame_shared = None
scan_thread = None
navigation_active = False

# 360-DEGREE HEAD SCANNING VARIABLES
head_turn = 'left_right'  # 'left_right' or 'up_down'
x_dis = 800  # Pan servo position (horizontal)
y_dis = 800  # Tilt servo position (vertical)
d_x = 15  # Horizontal step size
d_y = 15  # Vertical step size
scan_time_last = 0

# Servo range limits
SERVO_PAN_MIN = 400
SERVO_PAN_MAX = 1200
SERVO_TILT_MIN = 500
SERVO_TILT_MAX = 1100

def run_action_async(action_name):
    t = threading.Thread(
        target=AGC.runActionGroup,
        args=(action_name,),
        daemon=True
    )
    t.start()

# 360-DEGREE CONTINUOUS HEAD SCAN
def scan_360_head():
    """
    Performs continuous 360-degree head scan using servo sweep.
    First sweeps left-right, then adjusts up-down, repeats.
    Call this in your loop to continuously scan for objects.
    """
    global head_turn, x_dis, y_dis, d_x, d_y, scan_time_last
    
    current_time = time.time()
    
    # Update every 20ms (50Hz)
    if current_time - scan_time_last < 0.02:
        return
    
    scan_time_last = current_time
    
    try:
        if head_turn == 'left_right':
            x_dis += d_x
            # Reverse direction at limits
            if x_dis >= SERVO_PAN_MAX or x_dis <= SERVO_PAN_MIN:
                head_turn = 'up_down'
                d_x = -d_x
                print(f"[SERVO] Switching to UP_DOWN scan")
        
        elif head_turn == 'up_down':
            y_dis += d_y
            # Reverse direction at limits
            if y_dis >= SERVO_TILT_MAX or y_dis <= SERVO_TILT_MIN:
                head_turn = 'left_right'
                d_y = -d_y
                print(f"[SERVO] Switching to LEFT_RIGHT scan")
        
        # Apply servo positions
        rrc_board.pwm_servo_set_position(
            0.02,
            [
                [HEAD_PAN_SERVO, x_dis],
                [HEAD_TILT_SERVO, y_dis]
            ]
        )
    except Exception as e:
        print(f"[WARNING] 360 scan failed: {e}")

def reset_head_scan():
    """Reset head scan variables to initial state"""
    global head_turn, x_dis, y_dis, d_x, d_y
    head_turn = 'left_right'
    x_dis = 800
    y_dis = 800
    d_x = 15
    d_y = 15


def navigate_to_station(frame_getter, timeout=60, use_360_scan=False):
    """
    Navigate to QR station using continuous 360° head scan.
    
    Args:
        frame_getter: Function that returns current camera frame
        timeout: Maximum scan time in seconds
        use_360_scan: Legacy parameter (only 360° scan is supported now)
    
    Returns:
        Detected station data or None
    """

    global qr_scanning

    print("[INFO] Starting QR scan...")
    qr_scanning = True
    frames_without_qr = 0
    station_detected = None
    start_time = time.time()

    # Setup 360 continuous scanning
    reset_head_scan()
    print("[INFO] Starting 360° continuous head scan")

    while qr_scanning:

        # Timeout
        if time.time() - start_time > timeout:
            print("[INFO] Timeout")
            break

        frame = frame_getter()
        if frame is None:
            time.sleep(0.05)
            continue

        # Validate frame
        if frame.size == 0 or frame.shape[0] == 0 or frame.shape[1] == 0:
            time.sleep(0.05)
            continue
        
        # Convert BGR to RGB for pyzbar (pyzbar expects RGB)
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        
        barcodes = pyzbar.decode(frame_rgb)

        # -------- QR FOUND --------
        if barcodes:
            for barcode in barcodes:

                data = barcode.data.decode("utf-8")
                print(f"✓ QR Detected: {data}")

                # Center head servos
                rrc_board.pwm_servo_set_position(
                    0.3,
                    [[HEAD_PAN_SERVO, 800], [HEAD_TILT_SERVO, 800]]
                )
                time.sleep(0.3)

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
            # Use continuous 360-degree scan (every frame)
            scan_360_head()
            # Print progress every 50 frames
            if frames_without_qr % 50 == 0:
                print(f"[360-SCAN] Scanning (mode: {head_turn})... Frames: {frames_without_qr}")

        time.sleep(0.05)

    # Reset head
    print("[INFO] Resetting head to CENTER")
    rrc_board.pwm_servo_set_position(
        0.3,
        [[HEAD_PAN_SERVO, 800], [HEAD_TILT_SERVO, 800]]
    )
    time.sleep(0.3)
    reset_head_scan()

    print(f"[INFO] Scan complete → {station_detected}")
    return station_detected

if __name__ == "__main__":
    print("Use navigate_to_station(frame_getter) from main.py")
    
# ============ BACKGROUND THREAD FUNCTIONS ============
def _navigate_background_worker(timeout=60, use_360_scan=False):
    """
    Worker function that runs in background thread.
    Reads from shared current_frame_shared variable.
    """
    global qr_scanning, current_frame_shared, navigation_active

    def get_current_frame():
        if current_frame_shared is None:
            return None
        return current_frame_shared.copy()

    try:
        print(f"[DEBUG] Navigation worker started with use_360_scan={use_360_scan}")
        result = navigate_to_station(get_current_frame, timeout=timeout, use_360_scan=use_360_scan)
        print(f"[DEBUG] Navigation result: {result}")
        scan_result_queue.put(result)
    except Exception as e:
        print(f"[ERROR] Navigation thread error: {e}")
        import traceback
        traceback.print_exc()
        scan_result_queue.put(None)
    finally:
        navigation_active = False
        print("[DEBUG] Navigation worker ended")


def start_qr_navigation_async(timeout=60, use_360_scan=True):
    """
    START non-blocking QR navigation in background thread.
    Main camera loop continues to run!
    
    Args:
        timeout: Maximum scan time in seconds
        use_360_scan: Use 360° continuous scan (default True for best results)
    
    Returns:
        True if thread started, False if already running
    
    Usage in main loop:
        # Start once at beginning
        if not qr_navigate.navigation_active:
            qr_navigate.start_qr_navigation_async(timeout=60, use_360_scan=True)
        
        # In camera loop, keep updating shared frame
        qr_navigate.current_frame_shared = frame
        
        # Check if done
        if not qr_navigate.navigation_active:
            detected_station = qr_navigate.get_navigation_result()
    """
    global scan_thread, navigation_active

    if navigation_active:
        print("[WARNING] Navigation already running")
        return False

    # Clear queue
    try:
        while True:
            scan_result_queue.get_nowait()
    except queue.Empty:
        pass

    navigation_active = True
    scan_thread = threading.Thread(
        target=_navigate_background_worker,
        args=(timeout, use_360_scan),
        daemon=False
    )
    scan_thread.start()
    print("[INFO] QR Navigation started in background thread")
    return True


def get_navigation_result():
    """
    Get result from completed navigation.
    Returns None if still running or if no result.
    
    Usage:
        detected_station = qr_navigate.get_navigation_result()
        if detected_station:
            print(f"Found: {detected_station}")
    """
    try:
        result = scan_result_queue.get_nowait()
        return result
    except queue.Empty:
        return None


def stop_qr_navigation():
    """Stop active QR navigation"""
    global qr_scanning, navigation_active
    qr_scanning = False
    navigation_active = False
    print("[INFO] QR Navigation stopped")