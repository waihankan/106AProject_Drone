#Code adopted from
#https://github.com/GSNCodes/ArUCo-Markers-Pose-Estimation-Generation-Python/blob/main/detect_aruco_video.py

import numpy as np
import argparse
import time
import cv2
import sys

video = cv2.VideoCapture(0) #Video source coming from camera
deadzone_width = 400 #setting up deadzone
deadzone_height = 400
video.set(cv2.CAP_PROP_FRAME_WIDTH, 1920) #Camera sizes
video.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080) 

ARUCO_DICT = {
	"DICT_4X4_50": cv2.aruco.DICT_4X4_50,
	"DICT_4X4_100": cv2.aruco.DICT_4X4_100,
	"DICT_4X4_250": cv2.aruco.DICT_4X4_250,
	"DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
	"DICT_5X5_50": cv2.aruco.DICT_5X5_50,
	"DICT_5X5_100": cv2.aruco.DICT_5X5_100,
	"DICT_5X5_250": cv2.aruco.DICT_5X5_250,
	"DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
	"DICT_6X6_50": cv2.aruco.DICT_6X6_50,
	"DICT_6X6_100": cv2.aruco.DICT_6X6_100,
	"DICT_6X6_250": cv2.aruco.DICT_6X6_250,
	"DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
	"DICT_7X7_50": cv2.aruco.DICT_7X7_50,
	"DICT_7X7_100": cv2.aruco.DICT_7X7_100,
	"DICT_7X7_250": cv2.aruco.DICT_7X7_250,
	"DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
	"DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
	"DICT_APRILTAG_16h5": cv2.aruco.DICT_APRILTAG_16h5,
	"DICT_APRILTAG_25h9": cv2.aruco.DICT_APRILTAG_25h9,
	"DICT_APRILTAG_36h10": cv2.aruco.DICT_APRILTAG_36h10,
	"DICT_APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11
}



def aruco_display(corners, ids, rejected, image, tvecs = None, deadzone_coords = None):
	if deadzone_coords:
		negative_x,positive_x,negative_y,positive_y = deadzone_coords

	if len(corners) > 0:
		# flatten the ArUco IDs list
		ids = ids.flatten()
		# loop over the detected ArUCo corners
		for i,(markerCorner, markerID) in enumerate(zip(corners, ids)):
			# extract the marker corners (which are always returned in
			# top-left, top-right, bottom-right, and bottom-left order)
			corners = markerCorner.reshape((4, 2))
			(topLeft, topRight, bottomRight, bottomLeft) = corners
			# convert each of the (x, y)-coordinate pairs to integers
			topRight = (int(topRight[0]), int(topRight[1]))
			bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
			bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
			topLeft = (int(topLeft[0]), int(topLeft[1]))

			cv2.line(image, topLeft, topRight, (0, 255, 0), 2)
			cv2.line(image, topRight, bottomRight, (0, 255, 0), 2)
			cv2.line(image, bottomRight, bottomLeft, (0, 255, 0), 2)
			cv2.line(image, bottomLeft, topLeft, (0, 255, 0), 2)
			# compute and draw the center (x, y)-coordinates of the ArUco
			# marker
			cX = int((topLeft[0] + bottomRight[0]) / 2.0) #Center of the aruco markers
			cY = int((topLeft[1] + bottomRight[1]) / 2.0)
			cv2.circle(image, (cX, cY), 4, (0, 0, 255), -1)
			# draw the ArUco marker ID on the image
			cv2.putText(image, str(markerID),(topLeft[0], topLeft[1] - 10), cv2.FONT_HERSHEY_SIMPLEX,
				0.5, (0, 255, 0), 2)
			if tvecs is not None:
				current_tvec = tvecs[i][0] 
				x = current_tvec[0] #x distance from the aruco to camera center in meters
				y = current_tvec[1] #y distance from the aruco to camera center in meters
				z = current_tvec[2] #Distance from camera in meters, we can say within 2-2.5m do nothing, but if lower or higher then move forwards and backwards.
				location = ''

				if(cX > positive_x and cY > positive_y):
					location = 'Bottom Right' 
					#Fly downwards to the right
				elif (cX > positive_x and cY < negative_y):
					location = 'Top Right'
					#Fly to the top right
				elif (cX < negative_x and cY < negative_y):
					location = 'Top Left'
					#Fly to the top left
				elif (cX < negative_x and cY > positive_y):
					location = 'Bottom Left'
					#Fly to the bottom left
				elif (cX > positive_x and cY > negative_y and cY < positive_y):
					location = 'Right'
					#Fly to the right
				elif (cX < negative_x and cY > negative_y and cY < positive_y):
					location = 'Left'
					#Fly to the left
				elif (cX > negative_x and cX < positive_x and cY < negative_y):
					location = 'Up'
					#Fly up
				elif (cX > negative_x and cX < positive_x and cY > positive_y):
					location = 'Down'
					#fly down
				elif (cX >= negative_x and cX <= positive_x and cY >= negative_y and cY <= positive_y):
					location = 'In deadzone'
				print(f"Marker ID: {markerID} | X:{x:.3f} Y:{y:.3f} Z:{z:.3f} meters | {location}")
			# show the output image
	return image

camera_matrix = np.array([
        [1.43224559e+03, 0.00000000e+00, 9.60826529e+02],
        [0.00000000e+00, 1.43338093e+03, 5.35068560e+02],
        [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]
    ])
distortion = np.array([
        [-9.56033057e-03, 2.86911459e-01, -3.09675371e-04, 1.00210372e-03, -6.18334570e-01]
    ])
marker_size = 0.047




arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
arucoParams = cv2.aruco.DetectorParameters()
arucoDetector = cv2.aruco.ArucoDetector(arucoDict, arucoParams)



while True:
	ret, frame = video.read()
	if ret is False:
		break
	
	height, width, _ = frame.shape #Drawing rectangle to show where the deadzone is. Within the deadzone drone should not do anything
	center_x, center_y = width//2,height//2
	negative_x = center_x - deadzone_width//2
	positive_x = center_x + deadzone_width//2
	negative_y = center_y - deadzone_height//2
	positive_y = center_y + deadzone_height//2
	deadzone_coords = (negative_x,positive_x,negative_y,positive_y)

	corners, ids, rejected = arucoDetector.detectMarkers(frame)
	rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
        corners, marker_size, camera_matrix, distortion) #Grabbing translation vector to pass into display

	detected_markers = aruco_display(corners, ids, rejected, frame, tvecs, deadzone_coords)
	cv2.rectangle(frame,(negative_x,negative_y),(positive_x,positive_y),(0,0,255),2)
	cv2.imshow("Image", detected_markers)

	key = cv2.waitKey(1) & 0xFF
	if key == ord("q"):
	    break

cv2.destroyAllWindows()
video.release()