"""
QR Code Navigation Module
Robot finds and navigates to QR code using camera + head servos.
Flow: Scan (head sweeps) → Lock (QR found) → Track (keep centered) → Approach (walk forward)
Uses 2 threads: Vision thread (detect QR) + Move thread (control servos/walking)
"""

import cv2
import hiwonder.ActionGroupControl as AGC
from hiwonder import Controller, ros_robot_controller_sdk as rrc
import time
from pyzbar import pyzbar
import threading
import queue

# --- Hardware Init ---
rrc_board = rrc.Board()
ctl = Controller.Controller(rrc_board)

# --- Servo Config ---
HEAD_PAN_SERVO = 2   # Horizontal rotation (left-right)
HEAD_TILT_SERVO = 1  # Vertical tilt (up-down)

# Servo positions (PWM values)
PAN_CENTER = 1450    # Head straight ahead
TILT_CENTER = 1150   # Fixed table-level position
TILT_START = 1150
SERVO_PAN_MIN = 1000  # Max right
SERVO_PAN_MAX = 1900  # Max left
SERVO_TILT_MIN = 1150
SERVO_TILT_MAX = 1150

# ================================================================================
# SHARED STATE VARIABLES (Thread Communication)
# ================================================================================
# These variables are shared between the vision thread and movement thread.
# They allow the threads to coordinate the robot's behavior.
#
# IMPORTANT: In Python, simple variable reads/writes are atomic, so these
# don't need locks. For complex operations, use threading.Lock().
# ================================================================================

# Queue for passing final navigation result from worker thread to caller
scan_result_queue = queue.Queue()

# Shared camera frame for background navigation thread
current_frame_shared = None
navigation_active = False
qr_scanning = False

# QR tracking variables
object_center_x = -1   # QR center X position (-1 = not found)
object_width = 0       # QR width in pixels (larger = closer)
lost_frames = 0        # Frames since last QR detection
MAX_LOST_FRAMES = 30   # ~0.6s memory before "lost"
qr_locked = False      # True after first QR detection
last_known_x = -1      # Last known QR position

# Head scanning state
head_turn = 'left_right'
x_dis = PAN_CENTER
y_dis = TILT_START
d_x = 20   # Horizontal scan step
d_y = 15   # Vertical scan step (unused)
scan_time_last = 0


def run_action_async(action_name):
    """Run robot action in background thread (non-blocking)"""
    t = threading.Thread(target=AGC.runActionGroup, args=(action_name,), daemon=True)
    t.start()


# ================================================================================
# MOVEMENT CONTROL THREAD
# ================================================================================

def move():
    """
    Movement control thread - runs continuously.
    Case 1: QR locked & visible → adjust head/body, walk toward QR
    Case 2: Searching (not locked) → sweep head left-right
    Case 3: Locked but lost → keep moving toward last known position
    """
    global object_center_x, object_width, x_dis, y_dis, head_turn, d_x, d_y, qr_scanning, lost_frames, scan_time_last, qr_locked, last_known_x
    
    # Infinite loop - runs until program exits
    while True:
        # Only process movement when QR scanning is active
        if qr_scanning:
            
            # ================================================================
            # CASE 1: TARGET LOCKED AND TRACKING
            # ================================================================
            # Conditions:
            # - QR has been locked (first detection confirmed)
            # - Either QR is currently visible OR we haven't lost it too long
            if qr_locked and (object_center_x >= 0 or lost_frames < MAX_LOST_FRAMES):
                
                # ============================================================
                # STEP 1A: ADJUST HEAD TO TRACK QR POSITION
                # ============================================================
                # Only adjust head if we've recently seen the QR (not too many lost frames)
                # AND we currently have a valid detection
                if lost_frames < MAX_LOST_FRAMES // 2 and object_center_x >= 0:
                    # Calculate target head position based on where QR appears
                    target_x = PAN_CENTER  # Default: look straight ahead
                    
                    # If QR is on the left side of image (x < 240)
                    # Turn head slightly left to keep QR in view
                    if object_center_x < 240:
                        target_x = PAN_CENTER - 100  # Pan left
                    
                    # If QR is on the right side of image (x > 400)
                    # Turn head slightly right to keep QR in view
                    elif object_center_x > 400:
                        target_x = PAN_CENTER + 100  # Pan right
                    
                    # Apply the new head position
                    x_dis = target_x
                    # set_pwm_servo_pulse(servo_id, position, duration_ms)
                    ctl.set_pwm_servo_pulse(HEAD_PAN_SERVO, x_dis, 50)
                
                # ============================================================
                # STEP 1B: REALIGN BODY IF HEAD IS TURNED TOO FAR
                # ============================================================
                # If head is rotated more than 80 units from center,
                # rotate the body instead so head can return to center
                if abs(x_dis - PAN_CENTER) > 80:
                    if x_dis > PAN_CENTER:
                        # Head turned right → body should turn right
                        AGC.runActionGroup('turn_right')
                    else:
                        # Head turned left → body should turn left
                        AGC.runActionGroup('turn_left')
                    
                    # After body rotation, reset head to center position
                    x_dis, y_dis = PAN_CENTER, TILT_CENTER
                    ctl.set_pwm_servo_pulse(HEAD_TILT_SERVO, y_dis, 100)
                    ctl.set_pwm_servo_pulse(HEAD_PAN_SERVO, x_dis, 100)
                
                # ============================================================
                # STEP 2: FINE-TUNE BODY ALIGNMENT BASED ON QR PIXEL POSITION
                # ============================================================
                # Even with head centered, we may need small body adjustments
                # QR on left (x < 240) → turn body left
                elif object_center_x < 240 and object_center_x != -1:
                    run_action_async('turn_left')
                
                # QR on right (x > 400) → turn body right
                elif object_center_x > 400 and object_center_x != -1:
                    run_action_async('turn_right')
                
                # ============================================================
                # STEP 3: WALK FORWARD WHEN QR IS CENTERED
                # ============================================================
                else:
                    # Check if QR is still far away (width < 145 pixels)
                    # Larger width = QR is closer to camera
                    if object_width < 145 and object_width != 0:
                        # Still far → keep walking forward
                        run_action_async('go_forward')
                    
                    # QR is large enough → we've arrived at the station
                    elif object_width >= 145:
                        print("🎯 STATION REACHED")
                        qr_scanning = False     # Stop the scanning loop
                        qr_locked = False       # Reset lock state

            # ================================================================
            # CASE 2: INITIAL SEARCHING (Not locked yet, no QR visible)
            # ================================================================
            # Head scans left-right to find QR codes
            elif not qr_locked and object_center_x == -1:
                current_time = time.time()
                
                # Limit scan speed - only move every 30ms
                if current_time - scan_time_last > 0.03:
                    # ========================================================
                    # HORIZONTAL SCANNING (LEFT-RIGHT)
                    # ========================================================
                    # Increment/decrement x position by step size
                    x_dis += d_x
                    
                    # Reverse direction when hitting limits (ping-pong motion)
                    if x_dis >= SERVO_PAN_MAX or x_dis <= SERVO_PAN_MIN:
                        d_x = -d_x  # Reverse scanning direction
                    
                    # Keep vertical (tilt) position fixed at table level
                    # No vertical scanning - QR codes are at known height
                    y_dis = TILT_CENTER
                    ctl.set_pwm_servo_pulse(HEAD_TILT_SERVO, y_dis, 20)
                    ctl.set_pwm_servo_pulse(HEAD_PAN_SERVO, x_dis, 20)
                    scan_time_last = current_time
            
            # ================================================================
            # CASE 3: LOCKED BUT LOST VISION (Keep heading toward last known)
            # ================================================================
            # QR was locked but we've lost sight for too long
            # Instead of rescanning, continue toward last known position
            elif qr_locked and lost_frames >= MAX_LOST_FRAMES:
                # Use last known position to decide direction
                if last_known_x < 240:
                    run_action_async('turn_left')
                elif last_known_x > 400:
                    run_action_async('turn_right')
                else:
                    run_action_async('go_forward')
                
        # Sleep 20ms to prevent CPU overuse (50 iterations per second)
        time.sleep(0.02)


# ================================================================================
# START MOVEMENT THREAD
# ================================================================================
# Create and start the background movement thread
# daemon=True: Thread will automatically stop when main program exits
# ================================================================================
th = threading.Thread(target=move)
th.daemon = True
th.start()


def navigate_to_station(frame_getter, timeout=60):
    """
    Main navigation function - processes camera frames to find and approach QR.
    Args: frame_getter = function returning camera frame, timeout = max seconds
    Returns: QR code data string when reached, or None if timeout
    """
    global qr_scanning, object_center_x, object_width, lost_frames, qr_locked, last_known_x

    print("[INFO] Locking QR Station...")
    
    # Enable scanning mode (activates movement thread's tracking logic)
    qr_scanning = True
    qr_locked = False        # Reset lock state for new navigation
    start_time = time.time()
    station_data = None      # Will store QR code content when found

    # Main vision processing loop
    while qr_scanning:
        # ================================================================
        # TIMEOUT CHECK
        # ================================================================
        # Exit if we've been searching too long
        if time.time() - start_time > timeout:
            break

        # ================================================================
        # GET CAMERA FRAME
        # ================================================================
        # Call the provided function to get current camera image
        frame = frame_getter()
        if frame is None:
            continue  # Skip this iteration if no frame available
        
        # ================================================================
        # QR CODE DETECTION
        # ================================================================
        # Convert color space: OpenCV uses BGR, pyzbar expects RGB
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        
        # Decode all QR codes/barcodes in the image
        # pyzbar.decode returns a list of detected barcodes
        barcodes = pyzbar.decode(frame_rgb)

        # ================================================================
        # PROCESS DETECTED QR CODES
        # ================================================================
        if barcodes:
            # QR code found - reset the lost frame counter
            lost_frames = 0
            
            # Use the first detected barcode (assume only one target)
            barcode = barcodes[0]
            
            # Extract the QR code content (text/URL encoded in QR)
            station_data = barcode.data.decode("utf-8")
            
            # Extract bounding box coordinates
            # (x, y) = top-left corner, (w, h) = width and height
            (x, y, w, h) = barcode.rect
            
            # Calculate center X position for tracking
            # This tells movement thread where QR is horizontally
            object_center_x = x + (w // 2)
            
            # Store width for distance estimation
            # Larger width = QR is closer to camera
            object_width = w
            
            # Remember this position in case we lose sight temporarily
            last_known_x = object_center_x
            
            # ============================================================
            # LOCK ONTO QR AFTER FIRST DETECTION
            # ============================================================
            # Once we've seen the QR, we're "locked" - won't rescan
            if not qr_locked:
                qr_locked = True
                print("[INFO] 🎯 QR LOCKED! Starting navigation...")
            
            # Debug output every 10 frames (not every frame to reduce spam)
            if lost_frames % 10 == 0:
                print(f"[TRACK] QR detected: x={object_center_x}, width={object_width}, lost_frames={lost_frames}")
        
        else:
            # ============================================================
            # NO QR CODE DETECTED THIS FRAME
            # ============================================================
            # Increment lost frame counter
            lost_frames += 1
            
            # If lost for too many frames, reset detection variables
            # This allows movement thread to switch to "lost" behavior
            if lost_frames >= MAX_LOST_FRAMES:
                object_center_x = -1
                object_width = 0

        time.sleep(0.02)

    return station_data


# ================================================================================
# BACKGROUND NAVIGATION THREADING WRAPPERS
# ================================================================================
# These functions allow navigation to run in the background while the main
# program continues doing other tasks (like monitoring sensors).
# ================================================================================

def _navigate_background_worker(timeout=60):
    """Internal worker - runs navigation in background thread"""
    global current_frame_shared, navigation_active
    def get_current_frame():
        """Closure that returns a copy of the current shared frame."""
        return current_frame_shared.copy() if current_frame_shared is not None else None
    
    # Run the navigation with our frame getter
    result = navigate_to_station(get_current_frame, timeout=timeout)
    
    # Put the result in the queue for the main thread to retrieve
    scan_result_queue.put(result)
    
    # Mark navigation as complete
    navigation_active = False

def start_qr_navigation_async(timeout=60):
    """Start navigation in background. Returns True if started, False if already running."""
    global navigation_active
    
    # Prevent starting multiple navigations at once
    if navigation_active:
        return False
    
    # Mark navigation as active
    navigation_active = True
    
    # Start the background worker thread
    t = threading.Thread(target=_navigate_background_worker, args=(timeout,), daemon=True)
    t.start()
    return True

def get_navigation_result():
    """Get navigation result (non-blocking). Returns QR data or None if still running."""
    try: return scan_result_queue.get_nowait()
    except queue.Empty: return None