import cv2
import config
from vision.aruco_detector import ArucoVision

# Force config to match what we expect
config.TARGET_ID = 2

def test_aruco_vision_flip():
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Cannot open camera")
        return

    vision = ArucoVision()
    print("Press 'q' to quit. This should show a FLIPPED (Mirrored) image but still detect markers!")

    while True:
        ret, frame = cap.read()
        if not ret:
            break
            
        # Simulate Main Loop Flip
        frame = cv2.flip(frame, 1)
        
        # Process using our modified class
        targets = vision.process(frame)
        
        # Draw target details if found
        for t in targets:
            text = f"ID: {t.id} Area: {t.area:.4f} X: {t.center[0]:.2f}"
            cv2.putText(frame, text, (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
        cv2.imshow('Aruco Workaround Test', frame)
        if cv2.waitKey(1) == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    test_aruco_vision_flip()
