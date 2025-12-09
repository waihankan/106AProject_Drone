import cv2
import cv2.aruco as aruco
import numpy as np

print(f"OpenCV Version: {cv2.__version__}")

try:
    dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    parameters = aruco.DetectorParameters()
    print("Aruco module loaded successfully.")
    
    # Create a dummy image with a marker
    img = np.zeros((200, 200), dtype=np.uint8)
    # This might fail if drawMarker is not available in the same way
    try:
        aruco.drawMarker(dictionary, 0, 100, img, 1)
        print("Marker drawn successfully.")
        
        corners, ids, rejected = aruco.detectMarkers(img, dictionary, parameters=parameters)
        print(f"Detected markers: {ids}")
    except AttributeError as e:
        print(f"Error using Aruco functions: {e}")
        
except AttributeError as e:
    print(f"AttributeError: {e}. 'cv2.aruco' structure might be different.")
except Exception as e:
    print(f"An error occurred: {e}")
