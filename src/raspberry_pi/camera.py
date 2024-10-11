import socket
import cv2
import pickle

class CameraTransmitter:
    def __init__(self, host, port):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind((host, port))
        self.socket.setblocking(False)
        self.client = None
        self.camera = cv2.VideoCapture(0)
        self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)

    def send_frame(self):
        if self.client == None:
            try:
                _, addr = self.socket.recvfrom(32)
                self.client = (addr[0], int(addr[1]))
            except BlockingIOError:
                return

        ret, frame = self.camera.read()
        if ret:
            frame = cv2.rotate(frame, cv2.ROTATE_180)
            _, buffer = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 50])
            data = pickle.dumps(buffer)
            self.socket.sendto(data, self.client)

    def close(self):
        self.camera.release()
        self.socket.close()