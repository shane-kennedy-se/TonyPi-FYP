import cv2
import hiwonder.ActionGroupControl as AGC
from hiwonder import Controller, ros_robot_controller_sdk as rrc
import time
from pyzbar import pyzbar
import threading
import queue

# ============ INITIALIZATION ============
rrc_board = rrc.Board()
ctl = Controller.Controller(rrc_board)

# Servo IDs
HEAD_PAN_SERVO = 2
HEAD_TILT_SERVO = 1

# Configuration Constants (TonyPi Pro Standard)
# Adjust PAN_CENTER to 500 if 800 is not your robot's straight-ahead position
PAN_CENTER = 1450  
TILT_CENTER = 1450
SERVO_PAN_MIN = 1000
SERVO_PAN_MAX = 1900
SERVO_TILT_MIN = 1050
SERVO_TILT_MAX = 1500

# Thread-safe communication
scan_result_queue = queue.Queue()
current_frame_shared = None
navigation_active = False
qr_scanning = False

# Global Tracking States
object_center_x = -1
object_width = 0

# Head Scanning State
head_turn = 'left_right'
x_dis = PAN_CENTER
y_dis = TILT_CENTER
d_x = 20  
d_y = 20
scan_time_last = 0

# ============ HELPER FUNCTIONS ============

def run_action_async(action_name):
    """Runs action group in background so vision doesn't freeze."""
    t = threading.Thread(target=AGC.runActionGroup, args=(action_name,), daemon=True)
    t.start()

# ============ MOVEMENT THREAD (The "Transport.py" Logic) ============

def move():
    global object_center_x, object_width, x_dis, y_dis, head_turn, d_x, d_y, qr_scanning
    
    while True:
        if qr_scanning:
            # --- CASE 1: TARGET FOUND (Alignment & Walking) ---
            if object_center_x >= 0:
                # 1. TONYPI SECRET: If head is looking left/right, turn body to align first
                # This fixes the "head not centered" problem
                if abs(x_dis - PAN_CENTER) > 100:
                    if x_dis > PAN_CENTER:
                        print("↪ Head is looking right. Turning body right.")
                        AGC.runActionGroup('turn_right')
                    else:
                        print("↩ Head is looking left. Turning body left.")
                        AGC.runActionGroup('turn_left')
                    
                    # Gradually bring head back to center while body turns
                    x_dis = PAN_CENTER
                    y_dis = TILT_CENTER
                    ctl.set_pwm_servo_pulse(HEAD_TILT_SERVO, y_dis, 200)
                    ctl.set_pwm_servo_pulse(HEAD_PAN_SERVO, x_dis, 200)
                    time.sleep(0.5)

                # 2. Align body based on image center pixels
                elif object_center_x < 260: # Frame center is ~320
                    run_action_async('turn_left')
                elif object_center_x > 380:
                    run_action_async('turn_right')
                
                # 3. If aligned, move forward
                else:
                    if object_width < 130:
                        print("→ Moving forward to station")
                        run_action_async('go_forward')
                    else:
                        print("🎯 STATION REACHED")
                        qr_scanning = False # Stop loop
                        
            # --- CASE 2: SEARCHING (Zig-Zag Head Scan) ---
            elif object_center_x == -1:
                current_time = time.time()
                if current_time - scan_time_last > 0.03:
                    if head_turn == 'left_right':
                        x_dis += d_x
                        if x_dis >= SERVO_PAN_MAX or x_dis <= SERVO_PAN_MIN:
                            d_x = -d_x # Reverse direction
                            head_turn = 'up_down'
                    elif head_turn == 'up_down':
                        y_dis += d_y
                        if y_dis >= SERVO_TILT_MAX or y_dis <= SERVO_TILT_MIN:
                            d_y = -d_y
                        head_turn = 'left_right'
                    
                    ctl.set_pwm_servo_pulse(HEAD_TILT_SERVO, y_dis, 20)
                    ctl.set_pwm_servo_pulse(HEAD_PAN_SERVO, x_dis, 20)
                
        time.sleep(0.02)

# Start movement thread immediately
th = threading.Thread(target=move)
th.daemon = True
th.start()

# ============ VISION PROCESS ============

def navigate_to_station(frame_getter, timeout=60):
    global qr_scanning, object_center_x, object_width

    print("[INFO] Starting QR Station Navigation...")
    qr_scanning = True
    start_time = time.time()
    station_data = None

    while qr_scanning:
        if time.time() - start_time > timeout:
            break

        frame = frame_getter()
        if frame is None: continue
        
        # QR Detection
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        barcodes = pyzbar.decode(frame_rgb)

        if barcodes:
            barcode = barcodes[0]
            station_data = barcode.data.decode("utf-8")
            (x, y, w, h) = barcode.rect
            object_center_x = x + (w // 2)
            object_width = w
        else:
            object_center_x = -1 # Not seen, trigger search logic in move()

        time.sleep(0.02)

    return station_data

# ============ THREADING WRAPPERS ============

def _navigate_background_worker(timeout=60):
    global current_frame_shared, navigation_active
    def get_current_frame():
        return current_frame_shared.copy() if current_frame_shared is not None else None
    
    result = navigate_to_station(get_current_frame, timeout=timeout)
    scan_result_queue.put(result)
    navigation_active = False

def start_qr_navigation_async(timeout=60):
    global navigation_active
    if navigation_active: return False
    navigation_active = True
    t = threading.Thread(target=_navigate_background_worker, args=(timeout,), daemon=True)
    t.start()
    return True

def get_navigation_result():
    try: return scan_result_queue.get_nowait()
    except queue.Empty: return None