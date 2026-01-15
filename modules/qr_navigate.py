import cv2
import hiwonder.ActionGroupControl as AGC
from hiwonder import Controller, ros_robot_controller_sdk as rrc
import time
import threading
from pyzbar import pyzbar  # Use pyzbar for more reliable QR detection

rrc_board = rrc.Board()
Board = Controller.Controller(rrc_board)

# Head servo IDs (adjust if needed)
HEAD_PAN_SERVO = 8    # Left-Right rotation
HEAD_TILT_SERVO = 7   # Up-Down tilt

# QR Detection variables
qr_scanning = False

def rotate_head_to_search(direction='left'):
    """
    Rotate head to search for QR code.
    direction: 'left', 'right', or 'center'
    """
    try:
        if direction == 'left':
            Board.setPWMServoAngle(HEAD_PAN_SERVO, 120)  # Look left
            print("👀 Head: Looking LEFT")
        elif direction == 'right':
            Board.setPWMServoAngle(HEAD_PAN_SERVO, 40)   # Look right
            print("👀 Head: Looking RIGHT")
        else:  # center
            Board.setPWMServoAngle(HEAD_PAN_SERVO, 80)   # Center
            print("👀 Head: CENTER")
        time.sleep(0.3)
    except Exception as e:
        print(f"[WARNING] Head servo control failed: {e}")

def navigate_to_station(frame_getter, timeout=60):
    """
    Scan for QR code to find station using live frames from camera.
    Uses pyzbar for more reliable QR detection.
    
    Args:
        frame_getter: Function that returns current frame (e.g., lambda: latest_frame)
        timeout: Maximum time to scan for QR (seconds)
    
    Returns: station name (e.g., "A", "B", "C") or None if not found
    """
    global qr_scanning
    
    print("[INFO] Starting QR scan for station...")
    qr_scanning = True
    frames_without_qr = 0
    search_direction = 'left'
    station_detected = None
    start_time = time.time()

    while qr_scanning:
        # Timeout check
        if time.time() - start_time > timeout:
            print("[INFO] QR scan timeout")
            break
            
        # Get current frame from camera (already being captured by main.py)
        frame = frame_getter()
        if frame is None:
            time.sleep(0.05)
            continue

        # Use pyzbar to detect QR codes
        barcodes = pyzbar.decode(frame)
        
        if barcodes:
            for barcode in barcodes:
                data = barcode.data.decode("utf-8")
                print(f"✓ Detected QR: {data}")
                frames_without_qr = 0
                rotate_head_to_search('center')  # Center head when QR found

                # Get bounding box
                (x, y, w, h) = barcode.rect
                x_center = x + w // 2
                qr_width = w  # width of QR code
                frame_center = frame.shape[1] // 2

                # ---------- ALIGN BODY LEFT / RIGHT ----------
                if x_center < frame_center - 60:
                    print("← Turning LEFT to align body")
                    AGC.runActionGroup('WalkOneStep')
                    time.sleep(0.3)

                elif x_center > frame_center + 60:
                    print("→ Turning RIGHT to align body")
                    AGC.runActionGroup('WalkOneStep')
                    time.sleep(0.3)

                # ---------- MOVE FORWARD TO STATION ----------
                else:
                    if qr_width < 120:   # QR still far (< 120 pixels)
                        print("→ Approaching station...")
                        AGC.runActionGroup('WalkOneStep')
                        time.sleep(0.2)
                    else:
                        # QR is close enough - station reached!
                        print(f"✓✓✓ STATION REACHED: {data}")
                        station_detected = data
                        break

        else:
            # No QR detected - search by rotating head
            frames_without_qr += 1
            if frames_without_qr % 15 == 0:  # Search every ~15 frames (~0.75 sec)
                print(f"🔍 Searching... Head turning {search_direction}")
                if search_direction == 'left':
                    rotate_head_to_search('left')
                    search_direction = 'right'
                else:
                    rotate_head_to_search('right')
                    search_direction = 'left'

        time.sleep(0.05)

    qr_scanning = False
    print(f"[INFO] QR scan complete. Station: {station_detected}")
    return station_detected  # Return detected station (e.g., "A", "B", "C")

if __name__ == "__main__":
    print("QR Navigate module - use navigate_to_station(frame_getter) from main.py")
