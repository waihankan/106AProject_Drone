import cv2
import cv2.aruco as aruco
import numpy as np

# Use webcam 0
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("Cannot open camera")
    exit()

# Setup correct dictionary
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_250)
parameters = aruco.DetectorParameters()
detector = aruco.ArucoDetector(aruco_dict, parameters)

print("Press 'q' to quit.")

while True:
    ret, frame = cap.read()
    if not ret:
        print("Can't receive frame (stream end?). Exiting ...")
        break
    
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, rejected = detector.detectMarkers(gray)
    
    if ids is not None:
        print(f"Detected: {ids.flatten()}")
        aruco.drawDetectedMarkers(frame, corners, ids)
    
    cv2.imshow('Aruco Debug', frame)
    if cv2.waitKey(1) == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
