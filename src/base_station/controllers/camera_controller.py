import socket
import pickle
import cv2
from PIL import Image
import numpy as np

class CameraReceiver:
    def __init__(self, root, view, main_connection):
        self.root = root
        self.view = view
        self.main_connection = main_connection
        self.server_hostname = self.main_connection.getpeername()[0]
     
        self.is_running = False

        self.view.start_button.config(command=self.start)
        self.view.stop_button.config(command=self.stop)

    def start(self):
        self.main_connection.send("C 1".encode())
        response = self.main_connection.recv(32)
        try:
            data = response.decode().strip().split()
            if data[0] == "OK":
                self.camera_port = int(data[1])
            else:
                return None
        except:
            return None
        
        try:
            self.camera_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.camera_socket.setblocking(False)
            self.camera_socket.sendto("START".encode(), (self.server_hostname, self.camera_port))

        except:
            return None
        
        self.is_running = True
        self.view.start()
        self._receiver_loop()
        return True

    def _receiver_loop(self):
        if self.is_running:
            try:
                data, _ = self.camera_socket.recvfrom(65536)

                # print("Received frame")
                frame = pickle.loads(data)
                np_data = np.frombuffer(frame, np.uint8)
                frame = cv2.imdecode(np_data, cv2.IMREAD_COLOR)
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                image = Image.fromarray(frame)
                self.view.update_image(image)
            except BlockingIOError:
                pass

            except Exception as e:
                print(f"Socket error: {e}")
                self.stop()
                return
            self.root.after(10, self._receiver_loop)

    def stop(self):
        self.is_running = False
        if self.camera_socket:
            self.camera_socket.close()
        self.camera_socket = None
        self.main_connection.send("CAMERA STOP".encode())
        self.view.disable()
