import socket
import cv2
import pickle
import threading
import time

class CameraServer(threading.Thread):
    def __init__(self, hostname, port, client_hostname, fps):
        super().__init__(daemon=True)
        self.hostname = hostname
        self.port = port
        self.client_hostname = client_hostname
        self.client_port = None
        self.fps = fps

        self._stop_event = threading.Event()

        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind((self.hostname, self.port))
        print(f"[CAMERA SERVER] Listening on {self.hostname}:{self.port}")

        self.camera = cv2.VideoCapture(0)
        if not self.camera.isOpened():
            print("Error: Could not open video device")
            return
        self.camera.set(cv2.CAP_PROP_FPS, self.fps)


    def run(self):
        while not self._stop_event.is_set():
            if self.client_port is None:
                try:
                    data, addr = self.socket.recvfrom(1024)
                    if addr[0] != self.client_hostname:
                        print(f"[CAMERA SERVER] Received data from unknown client: {addr[0]} instead of {self.client_hostname}")
                        continue

                    if data.decode() == "START":
                        self.client_port = addr[1]
                        self.socket.sendto("OK".encode(), (self.client_hostname, self.client_port))
                except:
                    continue
            else:
                ret, frame = self.camera.read()
                if not ret:
                    continue

                if self.client_addr:
                    _, buffer = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 90])
                    data = pickle.dumps(buffer)
                    try:
                        self.socket.sendto(data, (self.client_hostname, self.client_port))
                        print("[CAMERA SERVER] Frame sent")
                    except Exception as e:
                        print(f"[CAMERA SERVER] Send failed: {e}")
                    # self.server_socket.sendto(data, (self.client_hostname, self.client_port))
                    
                time.sleep(1 / self.frame_rate)

    def stop(self):
        self._stop_event.set()
        self.server_socket.close()
        self.client_port = None
        print("[CAMERA SERVER] Server stopped.")