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

        try:
            self.camera = cv2.VideoCapture(0)
            if not self.camera.isOpened():
                print("Error: Could not open video device")
                return
        except Exception as e:
            self.camera = None
            return
        
        self.camera.set(cv2.CAP_PROP_FPS, self.fps)
        self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        # self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)


    def run(self):
        while not self._stop_event.is_set():
            if self.client_port is None:
                try:
                    data, addr = self.socket.recvfrom(1024)
                    if addr[0] != self.client_hostname:
                        print(f"[CAMERA SERVER] Received data from unknown client: {addr[0]} instead of {self.client_hostname}")
                        continue

                    if data.decode() == "START":
                        self.client_port = int(addr[1])
                        # self.socket.sendto("OK".encode(), (self.client_hostname, self.client_port))
                except:
                    continue
            else:
                ret, frame = self.camera.read()
                if not ret:
                    continue
                
                frame = cv2.rotate(frame, cv2.ROTATE_180)
                _, buffer = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 50])
                data = pickle.dumps(buffer)
                
                #self.socket.sendto(data, (self.client_hostname, self.client_port))
                try:
                    self.socket.sendto(data, (self.client_hostname, self.client_port))
                except Exception as e:
                    print(f"[CAMERA SERVER] Send failed: {e}")
                    self.client_port = None
                    self.stop()
                # self.server_socket.sendto(data, (self.client_hostname, self.client_port))
                
                
                

    def stop(self):
        self._stop_event.set()
        self.socket.close()
        self.client_port = None
        print("[CAMERA SERVER] Server stopped.")