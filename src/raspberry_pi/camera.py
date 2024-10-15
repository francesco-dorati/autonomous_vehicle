import socket
import cv2
import pickle
import time

class CameraTransmitter:
    def __init__(self, host, port):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind((host, port))
        self.socket.setblocking(False)
        self.client = None
        self.camera = cv2.VideoCapture(0)
        self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, 400)

    def send_frame(self):
        if self.client == None:
            try:
                _, addr = self.socket.recvfrom(32)
                self.client = (addr[0], int(addr[1]))
            except BlockingIOError:
                return
        t = time.time()
        ret, frame = self.camera.read()
        if ret:
            t_rotate = time.time()
            frame = cv2.rotate(frame, cv2.ROTATE_180)
            dt_rotate = (time.time() - t_rotate)*1000

            t_encode = time.time()
            _, buffer = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 100])
            dt_encode = (time.time() - t_encode)*1000

            t_dumps = time.time()
            data = pickle.dumps(buffer)
            dt_dumps = (time.time() - t_dumps)*1000

            t_send = time.time()
            self.socket.sendto(data, self.client)
            dt_send = (time.time() - t_send)*1000

            dt_ms = (time.time() - t)*1000
            print(f"CAMERA frame sent in {dt_ms:.1f} ms, rot: {dt_rotate:.1f} ms, enc: {dt_encode:.1f} ms, dumps: {dt_dumps:.1f} ms, send: {dt_send:.1f} ms")

    def close(self):
        self.camera.release()
        self.socket.close()