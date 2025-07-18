from flask import Flask, request
from datetime import datetime
import os
import socket
import subprocess

app = Flask(__name__)
UPLOAD_FOLDER = "pictures"

@app.route("/upload", methods=["POST"])
def upload_file():

    if "image" not in request.files:
        return "No image file provided", 400

    # Save image file in /pictures folder
    file = request.files["image"]
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"{timestamp}.jpg"
    filepath = os.path.join(UPLOAD_FOLDER, filename)
    file.save(filepath)

    file.stream.seek(0)
    fileBytes = file.read()
    fileLength = len(fileBytes)

    # Send to server
    RECOGNITION_SERVER_HOST = '127.0.0.1'
    RECOGNITION_SERVER_PORT = 65000

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.connect((RECOGNITION_SERVER_HOST, RECOGNITION_SERVER_PORT))
        s.sendall(fileLength.to_bytes(4, 'big') + fileBytes)
        response = s.recv(1024)
    
    return response.decode()

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000, debug=True)
