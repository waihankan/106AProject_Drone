import cv2

print("Scanning camera indices...")
for idx in range(5):  # try 0..4
    cap = cv2.VideoCapture(idx, cv2.CAP_DSHOW)  # use DirectShow on Windows
    if cap.isOpened():
        print(f"Camera index {idx} opened successfully.")
        ret, frame = cap.read()
        if ret:
            h, w, _ = frame.shape
            print(f"  -> Resolution: {w}x{h}")
        else:
            print("  -> Opened but failed to grab a frame.")
        cap.release()
    else:
        print(f"Camera index {idx} failed to open.")
