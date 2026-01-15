import cv2
import hiwonder.ActionGroupControl as AGC
from hiwonder import Controller, ros_robot_controller_sdk as rrc
import time

rrc_board = rrc.Board()
Board = Controller.Controller(rrc_board)

# Head servo IDs (adjust if needed)
HEAD_PAN_SERVO = 8    # Left-Right rotation
HEAD_TILT_SERVO = 7   # Up-Down tilt

def find_working_camera(max_index=3):
    for i in range(max_index):
        cap = cv2.VideoCapture(i)
        if cap.isOpened():
            cap.release()
            print(f"[INFO] Using camera index {i}")
            return i
        cap.release()
    return None

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

def navigate_to_station():
    """
    Scan for QR code to find station.
    Robot will:
    1. Search by rotating head side-to-side
    2. Align body to face QR
    3. Walk forward to reach station
    Returns: station name (e.g., "A", "B", "C") or None if not found
    """
    camera_index = find_working_camera()
    if camera_index is None:
        print("[ERROR] No camera found.")
        return None

    cap = cv2.VideoCapture(camera_index)
    detector = cv2.QRCodeDetector()

    print("[INFO] Scanning for station QR...")
    frames_without_qr = 0
    search_direction = 'left'
    station_detected = None

    while True:
        ret, frame = cap.read()
        if not ret:
            continue

        data, bbox, _ = detector.detectAndDecode(frame)

        if data and bbox is not None:
            print(f"✓ Detected QR: {data}")
            frames_without_qr = 0
            rotate_head_to_search('center')  # Center head when QR found

            pts = bbox[0]
            x_center = int((pts[0][0] + pts[2][0]) / 2)
            qr_width = abs(pts[0][0] - pts[1][0])  # distance indicator
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

        cv2.imshow("QR Navigation - Press ESC to exit", frame)
        if cv2.waitKey(1) == 27:  # ESC key
            print("[INFO] Navigation cancelled by user")
            break

        time.sleep(0.05)

    cap.release()
    cv2.destroyAllWindows()
    return station_detected  # Return detected station (e.g., "A", "B", "C")

if __name__ == "__main__":
    navigate_to_station()
