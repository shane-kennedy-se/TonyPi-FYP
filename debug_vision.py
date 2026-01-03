#!/usr/bin/python3
import cv2
import time
import sys
import threading
from modules import vision_module

# SHARED VARIABLES (Between the two threads)
latest_frame = None
latest_result = None
running = True
frame_lock = threading.Lock()
result_lock = threading.Lock()

def inference_worker(vision):
    """
    Background Thread: Runs YOLO as fast as it can, 
    updating 'latest_result' whenever it finishes.
    """
    global latest_frame, latest_result, running
    
    print("🧠 AI Brain Thread Started...")
    
    while running:
        # 1. Grab the latest frame safely
        img_to_process = None
        with frame_lock:
            if latest_frame is not None:
                img_to_process = latest_frame.copy()
        
        if img_to_process is not None:
            # 2. Run the heavy AI detection (This takes time!)
            # We run this on the copy so we don't block the video
            detection_data = vision.detect(img_to_process)
            
            # 3. Update the shared result safely
            with result_lock:
                latest_result = detection_data
        
        # Small sleep to prevent CPU overheating if camera is blank
        time.sleep(0.01)

def main():
    global latest_frame, running
    
    print("------------------------------------------")
    print("    SMOOTH THREADED VISION DEBUGGER       ")
    print("------------------------------------------")
    
    # 1. Initialize Vision Module
    vision = vision_module.VisionController()
    
    # 2. Open Camera
    cap = cv2.VideoCapture(0)
    frame_w = 640
    frame_h = 480
    cap.set(3, frame_w)
    cap.set(4, frame_h)
    
    if not cap.isOpened():
        print("❌ Camera failed to open.")
        return

    # 3. Start the AI Thread
    ai_thread = threading.Thread(target=inference_worker, args=(vision,))
    ai_thread.daemon = True # Kills thread if main program quits
    ai_thread.start()

    print("✅ System Online. Video should be smooth now.")
    print("   (Press 'q' to quit)")

    # 4. Main Loop (The "Eyes") - Runs as fast as possible
    while True:
        ret, frame = cap.read()
        if not ret: break

        # Update the shared frame so the AI can see it
        with frame_lock:
            latest_frame = frame

        # Get the LAST KNOWN result from the AI (don't wait for a new one)
        current_data = None
        with result_lock:
            current_data = latest_result

        # If we have data, draw it!
        if current_data:
            label, conf, box, cx = current_data
            
            if label:
                # --- Drawing Logic (Same as before) ---
                x1, y1, x2, y2 = box
                
                # Get Navigation Command
                cmd, error = vision.get_navigation_command(cx, frame_w)
                
                color = (0, 255, 0)
                status_text = f"CMD: {cmd} (Err: {error})"
                
                if cmd == "LOCKED":
                    color = (0, 0, 255)
                    cv2.putText(frame, "!!! TARGET LOCKED !!!", (cx - 80, y1 - 30), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
                    vision.run_action(label)
                    
                elif cmd == "TURN_LEFT":
                    cv2.arrowedLine(frame, (frame_w//2, frame_h//2), (frame_w//2 - 50, frame_h//2), (255, 255, 0), 3)
                elif cmd == "TURN_RIGHT":
                    cv2.arrowedLine(frame, (frame_w//2, frame_h//2), (frame_w//2 + 50, frame_h//2), (255, 255, 0), 3)

                cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
                cv2.putText(frame, f"{label} {conf:.2f}", (x1, y1 - 5), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
                cv2.putText(frame, status_text, (10, 40), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
            else:
                cv2.putText(frame, "Scanning...", (10, 40), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (100, 100, 100), 2)

        # Draw Center Line
        cv2.line(frame, (frame_w//2, 0), (frame_w//2, frame_h), (100, 100, 100), 1)
        
        # Show Frame
        cv2.imshow("Threaded Vision", frame)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            running = False # Tell thread to stop
            break

    cap.release()
    cv2.destroyAllWindows()
    running = False
    ai_thread.join()

if __name__ == "__main__":
    main()