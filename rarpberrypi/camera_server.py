import socket
import cv2
import pickle
import threading
import time

class CameraServer:
    def __init__(self, hostname, port, fps):
        self.hostname = hostname
        self.port = port
        self.fps = fps

        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind((self.hostname, self.port))
        print(f"[CAMERA SERVER] Listening on {self.hostname}:{self.port}")

        self.camera = cv2.VideoCapture(0)
        self.camera.set(cv2.CAP_PROP_FPS, self.fps)

        self.running = False

    def start(self):
        self.running = True
        self.thread = threading.Thread(target=self.stream)
        self.thread.start()

    def stop(self):
        self.running = False
        self.thread.join()

    def stream(self):
        while self.running:
            ret, frame = self.capture.read()
            if not ret:
                continue

            if self.client_addr:
                _, buffer = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 90])
                data = pickle.dumps(buffer)
                self.server_socket.sendto(data, self.client_addr)
                
            time.sleep(1 / self.frame_rate)