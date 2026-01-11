import cv2
import hiwonder.ActionGroupControl as AGC
from hiwonder import Controller, ros_robot_controller_sdk as rrc
import time

rrc_board = rrc.Board()
Board = Controller.Controller(rrc_board)

def find_working_camera(max_index=3):
    for i in range(max_index):
        cap = cv2.VideoCapture(i)
        if cap.isOpened():
            cap.release()
            print(f"[INFO] Using camera index {i}")
            return i
        cap.release()
    return None

def navigate_to_station():
    camera_index = find_working_camera()
    if camera_index is None:
        print("[ERROR] No camera found.")
        return

    cap = cv2.VideoCapture(camera_index)
    detector = cv2.QRCodeDetector()

    print("[INFO] Scanning for station QR...")

    while True:
        ret, frame = cap.read()
        if not ret:
            continue

        data, bbox, _ = detector.detectAndDecode(frame)

        if data and bbox is not None:
            print("Detected QR:", data)

            pts = bbox[0]
            x_center = int((pts[0][0] + pts[2][0]) / 2)
            qr_width = abs(pts[0][0] - pts[1][0])  # distance indicator
            frame_center = frame.shape[1] // 2

            # ---------- ALIGN LEFT / RIGHT ----------
            if x_center < frame_center - 40:
                print("Adjust LEFT")
                AGC.runActionGroup('turn_left')

            elif x_center > frame_center + 40:
                print("Adjust RIGHT")
                AGC.runActionGroup('turn_right')

            # ---------- MOVE FORWARD ----------
            else:
                if qr_width < 140:   # QR still far
                    print("Approaching station")
                    AGC.runActionGroup('go_forward')
                else:
                    print("Station reached:", data)
                    break   # STOP when close enough

        cv2.imshow("QR Navigation", frame)
        if cv2.waitKey(1) == 27:
            break

        time.sleep(0.05)

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    navigate_to_station()
