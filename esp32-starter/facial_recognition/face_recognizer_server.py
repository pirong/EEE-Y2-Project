import socket
import cv2
from face_recognizer import FaceRecognizer
import os
import numpy as np

HOST = '127.0.0.1'
PORT = 65000

recognizer = FaceRecognizer()
recognizer.load_database()

print("[+] Starting Facial Recognition Server")
with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.bind((HOST, PORT))
    s.listen()
    print(f"Listening on {HOST}:{PORT}")

    while True:
        conn, addr = s.accept()
        with conn:
            print(f"Connected by {addr}")
            length = int.from_bytes(conn.recv(4), 'big')
            data = b''
            while len(data) < length:
                packet = conn.recv(length - len(data))
                if not packet:
                    break
                data += packet

            # Decode image from bytes
            img_array = np.frombuffer(data, dtype=np.uint8)
            img = cv2.imdecode(img_array, cv2.IMREAD_COLOR)

            result = recognizer.recognize(img)

            if isinstance(result, list):
                for name in result:
                    if name.lower() in ["unknown", "database empty"]:
                        response = "Not recognised"
                    elif name.lower() == "no face":
                        response = "No face detected."
                    else:
                        response = "Recognised: " + name
            else:
                if result.lower() in ["unknown", "database empty"]:
                    response = "Not recognised"
                elif result.lower() == "no face":
                    response = "No face detected."
                else:
                    response = "Recognised: " + result

            conn.sendall(response.encode())
