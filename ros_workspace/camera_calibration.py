import numpy as np
import cv2
import glob

"""
After capturing images of a chessboard pattern, this script calibrates the camera.
"""

# Chessboard settings
CHESSBOARD_SIZE = (9, 6)   # IMPORTANT: The checkerboard we use is 9x6
SQUARE_SIZE = 22           # mm of each square (ours is 2.2 cm)

# Prepare object points based on the chessboard grid
objp = np.zeros((CHESSBOARD_SIZE[0] * CHESSBOARD_SIZE[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHESSBOARD_SIZE[0], 0:CHESSBOARD_SIZE[1]].T.reshape(-1, 2)
objp *= SQUARE_SIZE  # scale by square size

# Arrays to store points
objpoints = []  # 3D points
imgpoints = []  # 2D points

# Load images
images = glob.glob("calib_images/*.jpg")

for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Find corners
    found, corners = cv2.findChessboardCorners(gray, CHESSBOARD_SIZE, None)

    if found:
        objpoints.append(objp)

        corners2 = cv2.cornerSubPix(
            gray,
            corners,
            (11, 11),
            (-1, -1),
            (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        )

        imgpoints.append(corners2)

# Calibrate
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
    objpoints, imgpoints, gray.shape[::-1], None, None
)

# Print results
print("\n==== CAMERA CALIBRATION RESULTS ====")
print("Camera matrix (intrinsics):\n", mtx)
print("\nDistortion coefficients:\n", dist)
print("\nMean reprojection error:", ret)
print("====================================\n")
