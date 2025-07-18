import numpy as np
import cv2 as cv
from picamera2 import Picamera2
import requests
import serial
from time import sleep

SERVER_PATH = "http://184.72.78.19:5000/upload"
suspiciousIndividual = False

if __name__ == '__main__':
 
    detector = cv.FaceDetectorYN.create(
        'face_detection_yunet_2023mar.onnx',
        "",
        (1640, 1232),
        0.5,
        0.3,
        5000
    )
    
    tm = cv.TickMeter()

    picam2 = Picamera2()
    config = picam2.create_preview_configuration(main={"format": 'RGB888', "size": (1640, 1232)})  
    picam2.configure(config)

    picam2.start()

    while True:
        image = picam2.capture_array()

        # Inference
        tm.start()
        faces = detector.detect(image)
        tm.stop()

        # Send faces to server
        if faces[1] is not None:
            print("Faces Detected:", len(faces[1]))
            _, img_encoded = cv.imencode(".jpg", image)
            files = {"image": ("image.jpg", img_encoded.tobytes(), "image/jpeg")}
            response = requests.post(SERVER_PATH, files=files)
            responseText = response.text
         
            print(responseText)
            if "Recognized: " in responseText:
                suspiciousIndividual = True
                print("Suspicious individual detected")

            # Get pixel location of face
            # Individual positions are respective to facing the camera
            if suspiciousIndividual:
                horizontalCoords = faces[1][0][0] + 0.5*faces[1][0][2]
                verticalCoords = faces[1][0][1] + 0.5*faces[1][0][3]
                ser = serial.Serial("/dev/ttyS0", 9600)
                if horizontalCoords < 500:
                    message = "r"
                elif horizontalCoords > 1100:
                    message = "l"
                else:
                    message = "c"
                print(message)
                ser.write(message.encode())

        print("Estimated FPS:", 1/tm.getTimeSec())
        tm.reset()

    picam2.stop()

    cv.destroyAllWindows()
