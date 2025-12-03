from djitellopy import Tello
import numpy as np
import cv2
import time

# ==========================
# Tello Setup
# ==========================
SPEED_LR = 20      # left/right speed (cm/s)
SPEED_UD = 20      # up/down speed
SPEED_FB = 20      # forward/backward speed
TARGET_Z = 1.0     # desired distance to marker (meters)
Z_TOL = 0.2        # tolerance around target distance (meters)

tello = Tello()
print("Connecting to Tello...")
tello.connect()
battery = tello.get_battery()
print(f"Connected! Battery: {battery}%")

flying = False

# NOTE: We are using your laptop camera, so no need for tello.streamon()
# If you want to use Tello camera instead, you'll need a different setup
# and a new camera calibration.


# ==========================
# Video / ArUco Setup
# ==========================
video = cv2.VideoCapture(0)  # laptop camera
deadzone_width = 400
deadzone_height = 400
video.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
video.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

camera_matrix = np.array([
    [1.43224559e+03, 0.00000000e+00, 9.60826529e+02],
    [0.00000000e+00, 1.43338093e+03, 5.35068560e+02],
    [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]
])
distortion = np.array([
    [-9.56033057e-03, 2.86911459e-01, -3.09675371e-04, 1.00210372e-03, -6.18334570e-01]
])
marker_size = 0.047  # meters

arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
arucoParams = cv2.aruco.DetectorParameters()
arucoDetector = cv2.aruco.ArucoDetector(arucoDict, arucoParams)


def aruco_display(corners, ids, rejected, image, tvecs=None, deadzone_coords=None):
    """
    Draws ArUco markers and deadzone info.
    Returns:
        image (annotated),
        location (string or None),
        z (float or None)
    """
    location = None
    z_value = None

    if deadzone_coords:
        negative_x, positive_x, negative_y, positive_y = deadzone_coords

    if corners is not None and len(corners) > 0 and ids is not None:
        ids = ids.flatten()

        # We'll use the FIRST detected marker for control
        markerCorner = corners[0]
        markerID = ids[0]

        # Extract corners
        corners_reshaped = markerCorner.reshape((4, 2))
        (topLeft, topRight, bottomRight, bottomLeft) = corners_reshaped

        topRight = (int(topRight[0]), int(topRight[1]))
        bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
        bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
        topLeft = (int(topLeft[0]), int(topLeft[1]))

        # Draw marker box
        cv2.line(image, topLeft, topRight, (0, 255, 0), 2)
        cv2.line(image, topRight, bottomRight, (0, 255, 0), 2)
        cv2.line(image, bottomRight, bottomLeft, (0, 255, 0), 2)
        cv2.line(image, bottomLeft, topLeft, (0, 255, 0), 2)

        # Center of marker
        cX = int((topLeft[0] + bottomRight[0]) / 2.0)
        cY = int((topLeft[1] + bottomRight[1]) / 2.0)
        cv2.circle(image, (cX, cY), 4, (0, 0, 255), -1)

        cv2.putText(image, str(markerID),
                    (topLeft[0], topLeft[1] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (0, 255, 0), 2)

        if tvecs is not None:
            current_tvec = tvecs[0][0]  # first marker
            x = current_tvec[0]  # meters
            y = current_tvec[1]
            z = current_tvec[2]
            z_value = float(z)

            # Decide location relative to deadzone
            if deadzone_coords:
                if (cX > positive_x and cY > positive_y):
                    location = 'Bottom Right'
                elif (cX > positive_x and cY < negative_y):
                    location = 'Top Right'
                elif (cX < negative_x and cY < negative_y):
                    location = 'Top Left'
                elif (cX < negative_x and cY > positive_y):
                    location = 'Bottom Left'
                elif (cX > positive_x and negative_y < cY < positive_y):
                    location = 'Right'
                elif (cX < negative_x and negative_y < cY < positive_y):
                    location = 'Left'
                elif (negative_x < cX < positive_x and cY < negative_y):
                    location = 'Up'
                elif (negative_x < cX < positive_x and cY > positive_y):
                    location = 'Down'
                elif (negative_x <= cX <= positive_x and
                      negative_y <= cY <= positive_y):
                    location = 'In deadzone'

            text = f"ID:{markerID} X:{x:.3f} Y:{y:.3f} Z:{z:.3f}m | {location}"
            cv2.putText(image, text, (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)

    return image, location, z_value


print("""
=============================
   ARUCO TELLO CONTROLS
=============================
t : Takeoff
l : Land
q : Quit (lands if flying)

Drone behavior:
- If marker is ABOVE red box  -> drone flies UP
- BELOW red box               -> DOWN
- LEFT / RIGHT of red box     -> LEFT / RIGHT
- Diagonals (e.g. Top Right)  -> combined motion
- Distance (Z):
    Farther than target_z     -> forward
    Closer than target_z      -> backward
- In deadzone                 -> only distance control
=============================
""")

while True:
    ret, frame = video.read()
    if not ret:
        print("Failed to grab frame from camera.")
        break

    height, width, _ = frame.shape
    center_x, center_y = width // 2, height // 2
    negative_x = center_x - deadzone_width // 2
    positive_x = center_x + deadzone_width // 2
    negative_y = center_y - deadzone_height // 2
    positive_y = center_y + deadzone_height // 2
    deadzone_coords = (negative_x, positive_x, negative_y, positive_y)

    corners, ids, rejected = arucoDetector.detectMarkers(frame)

    # Estimate pose only if markers were found
    tvecs = None
    if corners is not None and len(corners) > 0:
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
            corners, marker_size, camera_matrix, distortion)

    frame, location, z = aruco_display(
        corners, ids, rejected, frame, tvecs, deadzone_coords
    )

    # Draw deadzone box
    cv2.rectangle(frame,
                  (negative_x, negative_y),
                  (positive_x, positive_y),
                  (0, 0, 255), 2)

    # ==========================
    # CONTROL LOGIC
    # ==========================
    lr = fb = ud = yaw = 0

    if flying:
        # Position-based control (left/right, up/down)
        if location is not None:
            # Horizontal
            if 'Right' in location:
                lr = SPEED_LR
            elif 'Left' in location:
                lr = -SPEED_LR

            # Vertical
            if 'Top' in location or location == 'Up':
                ud = SPEED_UD
            elif 'Bottom' in location or location == 'Down':
                ud = -SPEED_UD

        # Distance control (forward/back)
        if z is not None:
            if z > (TARGET_Z + Z_TOL):
                fb = SPEED_FB          # too far -> move forward
            elif z < (TARGET_Z - Z_TOL):
                fb = -SPEED_FB         # too close -> move backward

        # Send command to drone
        tello.send_rc_control(lr, fb, ud, yaw)

    # Show video
    cv2.imshow("Aruco Tello Control", frame)

    # Keyboard control inside OpenCV window
    key = cv2.waitKey(1) & 0xFF

    if key == ord('t'):
        if not flying:
            print("Taking off...")
            tello.takeoff()
            flying = True
    elif key == ord('l'):
        if flying:
            print("Landing...")
            tello.land()
            flying = False
            # Also stop motion
            tello.send_rc_control(0, 0, 0, 0)
    elif key == ord('q'):
        print("Quitting...")
        if flying:
            print("Landing before exit...")
            tello.land()
            flying = False
        break

# Cleanup
cv2.destroyAllWindows()
video.release()
tello.end()
print("Shutdown complete.")
