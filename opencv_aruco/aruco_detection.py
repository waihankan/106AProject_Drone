#from https://github.com/Ritabrata-Chakraborty/ArUco/blob/main/Aruco_Detection.py

import cv2
import numpy as np

ARUCO_DICT = {
	"DICT_4X4_250": cv2.aruco.DICT_4X4_250,

}

def detect_markers(image):
    
    aruco_type_list = []
    
    for aruco_type, dictionary_id in ARUCO_DICT.items():

        arucoDict = cv2.aruco.getPredefinedDictionary(dictionary_id)
        arucoParams = cv2.aruco.DetectorParameters()

        corners, ids, _ = cv2.aruco.detectMarkers(image, arucoDict, parameters=arucoParams)

        if len(corners) > 0:
            
            aruco_type_list.append(aruco_type)
            
            print(f"Markers detected using {aruco_type} dictionary")

            for markerCorner, markerId in zip(corners, ids.flatten()):
                corners_aruco = markerCorner.reshape((4, 2))
                (topLeft, topRight, bottomRight, bottomLeft) = corners_aruco

                cv2.polylines(image, [markerCorner.astype(int)], True, (0, 255, 0), 2)

                cX = int((topLeft[0] + bottomRight[0]) / 2)
                cY = int((topLeft[1] + bottomRight[1]) / 2)

                cv2.circle(image, (cX, cY), 5, (255, 0, 0), -1)
                cv2.putText(image, str(aruco_type) + " " + str(int(markerId)),
                            (int(topLeft[0] - 5), int(topLeft[1])), cv2.FONT_HERSHEY_COMPLEX, 0.5, (255, 0, 255))

            # break  # Stop iterating once markers are detected        
        # cv2.imshow("Detected Markers", image)
            
    return aruco_type_list

def pose_estimation(frame, aruco_dict_type, matrix_coefficients, distortion_coefficients):
    
    aruco_dict = cv2.aruco.getPredefinedDictionary(aruco_dict_type)
    parameters = cv2.aruco.DetectorParameters()

    corners, ids, rejected = cv2.aruco.detectMarkers(image, aruco_dict, parameters=parameters)

    if len(corners) > 0:
        for i in range(0, len(ids)):
           
            rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(corners[i], 0.047, matrix_coefficients, distortion_coefficients) #change for aruco size
            
            cv2.aruco.drawDetectedMarkers(frame, corners) 
            distance_z = tvec[0][0][2] 
            print(f"Marker ID {ids[i][0]} Distance (Z): {distance_z:.3f} meters")
            cv2.drawFrameAxes(frame, matrix_coefficients, distortion_coefficients, rvec, tvec, 0.03) 
             
    return frame

if __name__ == "__main__":

    image_path = r"arucoMarkers/singlemarkersoriginal.jpg"

    intrinsic_camera = np.array([
        [1.43224559e+03, 0.00000000e+00, 9.60826529e+02],
        [0.00000000e+00, 1.43338093e+03, 5.35068560e+02],
        [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]
    ])
    distortion = np.array([
        [-9.56033057e-03, 2.86911459e-01, -3.09675371e-04, 1.00210372e-03, -6.18334570e-01]
    ])

    cap = cv2.VideoCapture(0)

    while cap.isOpened():
        ret, image = cap.read()

        for aruco_type in detect_markers(image):
             image = pose_estimation(image, ARUCO_DICT[aruco_type], intrinsic_camera, distortion)
             
        cv2.imshow('Estimated Pose', image)
                
        if cv2.waitKey(50) & 0xFF == 27: #ESC to exit
            break

    cap.release()
    cv2.destroyAllWindows()