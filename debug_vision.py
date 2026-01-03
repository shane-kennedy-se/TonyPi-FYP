#!/usr/bin/python3
import cv2
import time
import sys
# Import our new module
from modules import vision_module

def main():
    print("------------------------------------------")
    print("      YOLO VISION DEBUGGER (NO ACTIONS)   ")
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

    print("✅ System Online. Point camera at cardboard.")
    print("   (Press 'q' to quit)")

    while True:
        ret, frame = cap.read()
        if not ret: break

        # A. Get Detection from YOLO
        label, conf, box, cx = vision.detect(frame)

        if label:
            x1, y1, x2, y2 = box
            
            # B. Get Navigation Command (Locking Logic)
            cmd, error = vision.get_navigation_command(cx, frame_w)
            
            # C. Draw Visuals
            color = (0, 255, 0) # Green = Tracking
            status_text = f"CMD: {cmd} (Err: {error})"
            
            if cmd == "LOCKED":
                color = (0, 0, 255) # Red = Locked
                cv2.putText(frame, "!!! TARGET LOCKED !!!", (cx - 80, y1 - 30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
                
                # Report "Fake" Action Trigger
                vision.run_action(label)
                
            elif cmd == "TURN_LEFT":
                # Draw Arrow pointing Left
                cv2.arrowedLine(frame, (frame_w//2, frame_h//2), (frame_w//2 - 50, frame_h//2), (255, 255, 0), 3)
            
            elif cmd == "TURN_RIGHT":
                # Draw Arrow pointing Right
                cv2.arrowedLine(frame, (frame_w//2, frame_h//2), (frame_w//2 + 50, frame_h//2), (255, 255, 0), 3)

            # Draw Bounding Box & Label
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
            cv2.putText(frame, f"{label} {conf:.2f}", (x1, y1 - 5), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
            
            # Draw Center Line (Crosshair)
            cv2.line(frame, (frame_w//2, 0), (frame_w//2, frame_h), (100, 100, 100), 1)

        else:
            cv2.putText(frame, "Scanning...", (10, 40), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (100, 100, 100), 2)

        cv2.imshow("Vision Debugger", frame)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()