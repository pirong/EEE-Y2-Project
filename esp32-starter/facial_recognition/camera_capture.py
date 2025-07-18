import time
import cv2

def capture_frame(cap, flush_duration=0.2):
    start = time.time()
    while time.time() - start < flush_duration:
        cap.grab()
    ret, frame = cap.read()
    return frame if ret else None