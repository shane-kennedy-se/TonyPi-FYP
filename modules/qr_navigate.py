"""
================================================================================
QR CODE NAVIGATION MODULE - Visual Homing System
================================================================================
This module enables the TonyPi robot to navigate towards a QR code target
using computer vision and servo-controlled head movement.

SYSTEM OVERVIEW:
----------------
The robot uses a camera mounted on its head to:
1. SCAN: Look around by moving head servos to find QR codes
2. LOCK: Once a QR code is detected, lock onto it
3. TRACK: Keep the QR code centered in the camera view
4. NAVIGATE: Walk towards the QR code until close enough

NAVIGATION ALGORITHM:
---------------------
The robot uses a state machine with three main states:
1. SEARCHING: Head scans left-right to find QR code
2. TRACKING: Robot adjusts position to keep QR centered
3. APPROACHING: Robot walks forward until close enough

COORDINATE SYSTEM:
------------------
Camera image is 640x480 pixels:
- X-axis: 0 (left) to 640 (right), center = 320
- QR code position used to determine turn direction:
  - x < 240: QR is on the left → turn left
  - x > 400: QR is on the right → turn right
  - 240 ≤ x ≤ 400: QR is centered → walk forward

SERVO SYSTEM:
-------------
- HEAD_PAN_SERVO (Servo 2): Rotates head left-right (horizontal)
  - 1000 = Full right, 1450 = Center, 1900 = Full left
- HEAD_TILT_SERVO (Servo 1): Tilts head up-down (vertical)
  - Fixed at 1150 for table-level scanning

MULTI-THREADING:
----------------
This module uses two concurrent threads:
1. Main Thread: Vision processing - reads camera, detects QR codes
2. Move Thread: Movement control - controls servos and walking

This separation allows the robot to process vision AND move simultaneously.

Author: FYP Project
================================================================================
"""

import cv2                                              # OpenCV for image processing
import hiwonder.ActionGroupControl as AGC              # Pre-recorded robot movements
from hiwonder import Controller, ros_robot_controller_sdk as rrc  # Hardware control
import time
from pyzbar import pyzbar                               # QR code/barcode detection library
import threading                                        # For concurrent execution
import queue                                            # Thread-safe communication

# ================================================================================
# HARDWARE INITIALIZATION
# ================================================================================
# Initialize the robot controller board communication
# rrc_board: Low-level hardware interface to the TonyPi controller board
# ctl: High-level controller for servo and motor commands
# ================================================================================
rrc_board = rrc.Board()
ctl = Controller.Controller(rrc_board)

# ================================================================================
# SERVO CONFIGURATION
# ================================================================================
# The TonyPi robot has two head servos for camera positioning:
#
# HEAD_PAN_SERVO (Servo 2): Horizontal rotation (left-right)
#   - Allows robot to look around without moving body
#   - PWM range: 500-2500μs (microseconds)
#
# HEAD_TILT_SERVO (Servo 1): Vertical tilt (up-down)
#   - Allows robot to look up at faces or down at floor
#   - Fixed at table level for this application
# ================================================================================
HEAD_PAN_SERVO = 2      # Servo ID for horizontal head rotation
HEAD_TILT_SERVO = 1     # Servo ID for vertical head tilt

# ================================================================================
# PAN/TILT POSITION CONSTANTS
# ================================================================================
# PWM (Pulse Width Modulation) values control servo positions:
# - Lower values: Rotate one direction
# - Higher values: Rotate opposite direction
# - Middle values: Center position
#
# These values are calibrated for the TonyPi robot's specific servos
# ================================================================================
PAN_CENTER = 1450       # Head looking straight ahead (horizontal center)
TILT_CENTER = 1150      # Fixed stable position - looking at table level
TILT_START = 1150       # Same as center - no vertical scanning needed

# Servo movement limits to prevent mechanical damage
SERVO_PAN_MIN = 1000    # Maximum right rotation
SERVO_PAN_MAX = 1900    # Maximum left rotation
SERVO_TILT_MIN = 1150   # Fixed tilt (no vertical movement)
SERVO_TILT_MAX = 1150   # Fixed tilt (no vertical movement)

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

# Flags to control navigation state
navigation_active = False    # True when background navigation is running
qr_scanning = False          # True when actively looking for/tracking QR

# ================================================================================
# QR CODE TRACKING VARIABLES
# ================================================================================
# These variables track the QR code's position and state
# ================================================================================
object_center_x = -1    # X-coordinate of QR code center (-1 = not detected)
object_width = 0        # Width of QR code in pixels (larger = closer)
lost_frames = 0         # Counter: how many frames since we last saw QR
MAX_LOST_FRAMES = 30    # ~0.6 seconds (at 50fps) of "memory" to maintain lock
qr_locked = False       # True after first QR detection (prevents rescanning)
last_known_x = -1       # Remember last QR position when temporarily lost

# ================================================================================
# HEAD SCANNING STATE VARIABLES
# ================================================================================
# These control the head's scanning motion when searching for QR codes
# ================================================================================
head_turn = 'left_right'    # Scanning direction mode
x_dis = PAN_CENTER          # Current horizontal servo position
y_dis = TILT_START          # Current vertical servo position (fixed)
d_x = 20                    # Step size for horizontal scanning (speed)
d_y = 15                    # Step size for vertical scanning (not used)
scan_time_last = 0          # Timestamp of last scan movement


# ================================================================================
# HELPER FUNCTIONS
# ================================================================================

def run_action_async(action_name):
    """
    Run a robot action group in a separate thread (non-blocking).
    
    Action groups are pre-recorded sequences of servo movements saved as files.
    Examples: 'go_forward', 'turn_left', 'turn_right'
    
    Running asynchronously allows:
    - Vision processing to continue while robot moves
    - Smoother, more responsive behavior
    - Multiple actions to be queued
    
    Args:
        action_name (str): Name of the action group file (without extension)
    
    Note:
        daemon=True means the thread will be killed when main program exits
    """
    t = threading.Thread(target=AGC.runActionGroup, args=(action_name,), daemon=True)
    t.start()


# ================================================================================
# MOVEMENT CONTROL THREAD
# ================================================================================

def move():
    """
    Background thread that continuously controls robot movement.
    
    This function runs in an infinite loop and handles three cases:
    
    CASE 1: TARGET LOCKED AND TRACKING (qr_locked = True, QR visible)
    -----------------------------------------------------------------
    - Adjust head servo to keep QR centered
    - If head is turned too far, rotate body to realign
    - Fine-tune body rotation based on QR position in image
    - Walk forward when QR is centered
    - Stop when QR is close enough (width >= 145 pixels)
    
    CASE 2: INITIAL SEARCHING (not locked yet, no QR visible)
    ---------------------------------------------------------
    - Sweep head left-right to scan for QR codes
    - Keep vertical position fixed at table level
    - When QR is found, vision thread will lock onto it
    
    CASE 3: LOCKED BUT TEMPORARILY LOST VISION
    -------------------------------------------
    - Continue moving toward last known position
    - This handles brief visibility loss during walking (camera shake)
    - Prevents robot from oscillating between search and track modes
    
    Thread Safety:
    - Uses global variables for state (atomic reads/writes)
    - Movement commands are queued, not immediate
    - 20ms sleep prevents CPU overuse
    """
    # Declare all global variables this function modifies
    global object_center_x, object_width, x_dis, y_dis, head_turn
    global d_x, d_y, qr_scanning, lost_frames, scan_time_last
    global qr_locked, last_known_x
    
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
                    
                    # Apply servo positions
                    # Duration 20ms for smooth, quick movements
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


# ================================================================================
# VISION PROCESSING FUNCTION
# ================================================================================

def navigate_to_station(frame_getter, timeout=60):
    """
    Main navigation function that processes camera frames to find and navigate to QR codes.
    
    This function:
    1. Enables QR scanning mode
    2. Continuously reads camera frames
    3. Detects QR codes using pyzbar library
    4. Updates tracking variables (shared with move() thread)
    5. Returns the QR code data when destination is reached
    
    Args:
        frame_getter (callable): Function that returns the current camera frame
                                 Must return a numpy array (BGR image) or None
        timeout (int): Maximum seconds to search before giving up
    
    Returns:
        str: QR code data string if station reached
        None: If timeout or no QR found
    
    QR Code Detection:
    ------------------
    Uses pyzbar library to decode QR codes from images:
    1. Convert BGR (OpenCV format) to RGB (pyzbar format)
    2. pyzbar.decode() returns list of detected barcodes
    3. Each barcode has:
       - data: The encoded text/URL
       - rect: Bounding box (x, y, width, height)
    
    Frame Rate:
    -----------
    Loop runs at ~50fps (20ms sleep) for responsive tracking
    """
    # Declare global variables this function modifies
    global qr_scanning, object_center_x, object_width
    global lost_frames, qr_locked, last_known_x

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

        # Sleep 20ms between frames (~50 fps processing rate)
        time.sleep(0.02)

    # Return the QR code data (or None if not found/timeout)
    return station_data


# ================================================================================
# BACKGROUND NAVIGATION THREADING WRAPPERS
# ================================================================================
# These functions allow navigation to run in the background while the main
# program continues doing other tasks (like monitoring sensors).
# ================================================================================

def _navigate_background_worker(timeout=60):
    """
    Internal worker function that runs navigation in a background thread.
    
    This wraps navigate_to_station() for use with threading:
    1. Creates a frame getter function that reads from shared global
    2. Runs the navigation
    3. Puts result in queue for retrieval
    4. Marks navigation as inactive
    
    Args:
        timeout (int): Maximum seconds to search
    
    Note:
        This function is not meant to be called directly.
        Use start_qr_navigation_async() instead.
    """
    global current_frame_shared, navigation_active
    
    def get_current_frame():
        """
        Closure that returns a copy of the current shared frame.
        
        Returns a copy to prevent race conditions where the frame
        might be modified while navigation is reading it.
        """
        return current_frame_shared.copy() if current_frame_shared is not None else None
    
    # Run the navigation with our frame getter
    result = navigate_to_station(get_current_frame, timeout=timeout)
    
    # Put the result in the queue for the main thread to retrieve
    scan_result_queue.put(result)
    
    # Mark navigation as complete
    navigation_active = False


def start_qr_navigation_async(timeout=60):
    """
    Start QR navigation in a background thread.
    
    Use this to begin navigation without blocking the main program.
    The main program can continue running other tasks (sensor monitoring,
    user interface, etc.) while navigation happens in the background.
    
    Args:
        timeout (int): Maximum seconds to search for QR code
    
    Returns:
        bool: True if navigation started successfully
              False if navigation is already running
    
    Usage Example:
        # Start navigation
        if start_qr_navigation_async():
            # Do other tasks while navigating...
            
            # Check if navigation is complete
            result = get_navigation_result()
            if result is not None:
                print(f"Arrived at: {result}")
    
    Note:
        While navigation is active, update current_frame_shared with
        new camera frames so the navigation can process them.
    """
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
    """
    Retrieve the result of background navigation (non-blocking).
    
    Call this periodically to check if navigation has completed.
    
    Returns:
        str: QR code data if navigation completed successfully
        None: If navigation is still running or failed
    
    Usage:
        while True:
            result = get_navigation_result()
            if result is not None:
                print(f"Destination reached: {result}")
                break
            # Do other tasks...
            time.sleep(0.1)
    
    Note:
        This is non-blocking - it returns immediately even if
        navigation is not yet complete.
    """
    try:
        # get_nowait() returns immediately, raises Empty if queue is empty
        return scan_result_queue.get_nowait()
    except queue.Empty:
        return None