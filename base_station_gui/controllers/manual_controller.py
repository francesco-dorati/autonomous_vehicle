import socket
import select
import pickle
import cv2
from PIL import Image
import numpy as np

MANUAL_CONTROL_FREQ = 10
MANUAL_CONTROL_MS = int((1/MANUAL_CONTROL_FREQ)*1000)

class ManualController:
    def __init__(self, root, view, main_connection):
        self.root = root
        self.view = view

        self.controls_sender = ControlsSender(self.root, self.view.controls_frame, main_connection)
        self.data_receiver = DataReceiver(self.root, self.view.data_frame, main_connection)
        self.camera_receiver = CameraReceiver(self.root, self.view.camera_frame, main_connection)


        ok = self.controls_sender.start()
        if not ok:
            self.view.controls_frame.disable()
        
        ok = self.data_receiver.start()
        if not ok:
            self.view.data_frame.disable()
    def stop():
        pass

class ControlsSender:
    def __init__(self, root, view, main_connection):
        self.root = root
        self.view = view
        self.main_connection = main_connection
        self.server_hostname = self.main_connection.getpeername()[0]

        self.controls_port = None
        self.controls_socket = None
        self.is_running = False
        
        self.keyboard_buffer = []

        self.view.start_button.config(command=self.start)
        self.view.stop_button.config(command=self.stop)

    def start(self):
        self.main_connection.send("MANUAL START".encode())
        response = self.main_connection.recv(1024)
        try:
            data = response.decode().strip().split()
            if data[0] == "OK":
                self.controls_port = int(data[1])
            else:
                return None
        except:
            return None
        
        try:
            self.controls_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        except:
            return None
        
        # KEYBINDINGS
        self.view.focus_set()
        self.view.bind("<KeyPress>", self._key_pressed)
        self.view.bind("<KeyRelease>", self._key_released)

        self.is_running = True
        self.view.enable()
        self._sender_loop()
        return True
    
    def _sender_loop(self):
        # refine buffer
        if self.is_running:
            if 'f' in self.keyboard_buffer and 'b' in self.keyboard_buffer:
                self.keyboard_buffer.remove('f')
                self.keyboard_buffer.remove('b')
            if 'l' in self.keyboard_buffer and 'r' in self.keyboard_buffer:
                self.keyboard_buffer.remove('l')
                self.keyboard_buffer.remove('r')
            
            s = "".join(self.keyboard_buffer)
            # print("Sent:", s, " to ", self.server_hostname, ":", self.manual_port)
            try:
                self.controls_socket.sendto(s.encode(), (self.server_hostname, self.controls_port))
            except:
                print("ERROR, stopping")
                self.stop()
                return

            self.root.after(MANUAL_CONTROL_MS, self._sender_loop)
    
    def stop(self):
        self.is_running = False
        self.view.unbind("<KeyPress>")
        self.view.unbind("<KeyRelease>")
        self.controls_socket.close()
        self.controls_socket = None
        self.main_connection.send("MANUAL STOP".encode())
        self.view.disable()
  
    def _key_pressed(self, event):
        key = event.keysym
        if key == 'w': # FORWARD
            if not 'f' in self.keyboard_buffer:
                self.keyboard_buffer.append('f')
            self.view.forward(True)
        elif key == 's': # BACKWARD
            if not 'b' in self.keyboard_buffer:
                self.keyboard_buffer.append('b')
            self.view.backward(True)
        elif key == 'a':   # LEFT
            if not 'l' in self.keyboard_buffer:
                self.keyboard_buffer.append('l')
            self.view.left(True)
        elif key == 'd':  # RIGHT
            if not 'r' in self.keyboard_buffer:
                self.keyboard_buffer.append('r')
            self.view.right(True)

    def _key_released(self, event):
        key = event.keysym
        if key == 'w':
            if 'f' in self.keyboard_buffer:
                self.keyboard_buffer.remove('f')
            self.view.forward(False)
        elif key == 's':
            if 'b' in self.keyboard_buffer:
                self.keyboard_buffer.remove('b')
            self.view.backward(False)
        elif key == 'a':
            if 'l' in self.keyboard_buffer:
                self.keyboard_buffer.remove('l')
            self.view.left(False)
        elif key == 'd':
            if 'r' in self.keyboard_buffer:
                self.keyboard_buffer.remove('r')
            self.view.right(False)



class DataReceiver:
    def __init__(self, root, view, main_connection):
        self.root = root
        self.view = view
        self.main_connection = main_connection
        self.server_hostname = self.main_connection.getpeername()[0]

        self.data_port = None
        self.data_socket = None
        self.is_running = False

    def start(self):
        self.main_connection.send("DATA START".encode())
        response = self.main_connection.recv(1024)
        try:
            data = response.decode().strip().split()
            if data[0] == "OK":
                self.data_port = int(data[1])
            else:
                return None
        except:
            return None
        
        # SEND DATAT RATE
        
        try:
            self.data_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.data_socket.sendto("START".encode(), (self.server_hostname, self.data_port))
        except:
            return None
        
        self.is_running = True
        self._receiver_loop()
        return True
    
    def _receiver_loop(self):
        if self.is_running:
            try:
                ready = select.select([self.data_socket], [], [], 0.1)
                if ready[0]:
                    data, _ = self.data_socket.recvfrom(1024)
                    vel, pos = self._parse_data(data.decode())
                    self.view.update(vel, pos)

            except:
                self.stop()
                return
            self.root.after(100, self._receiver_loop)

    def _parse_data(self, data):
        data = data.strip().split()
        vel = [float(data[0]), float(data[1])]
        pos = [float(data[2]), float(data[3]), float(data[4])]
        return vel, pos

    def stop(self):
        self.is_running = False
        self.data_socket.close()
        self.data_socket = None
        self.main_connection.send("DATA STOP".encode())
        self.view.disable()

class CameraReceiver:
    def __init__(self, root, view, main_connection):
        self.root = root
        self.view = view
        self.main_connection = main_connection
        self.server_hostname = self.main_connection.getpeername()[0]
     
        self.data_port = None
        self.data_socket = None
        self.is_running = False

        self.view.start_button.config(command=self.start)
        self.view.stop_button.config(command=self.stop)

    def start(self):
        self.main_connection.send("CAMERA START".encode())
        response = self.main_connection.recv(1024)
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
                ready = select.select([self.camera_socket], [], [], 0.1)
                if ready[0]:
                    data, _ = self.camera_socket.recvfrom(1024)
                    frame = pickle.loads(data)
                    np_data = np.frombuffer(frame, np.uint8)
                    frame = cv2.imdecode(np_data, cv2.IMREAD_COLOR)
                    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                    image = Image.fromarray(frame)
                    self.view.update_image(image)

            except:
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

    # def _get_data_port(self) -> int:
    #     # get manual port
    #     self.main_connection.send("DATA START".encode())
    #     response = self.main_connection.recv(1024)
    #     try:
    #         data = response.decode().strip().split()
    #         if data[0] == "OK":
    #             return int(data[1])
    #         else:
    #             return None
    #     except:
    #         return None
        
    # def _get_manual_port(self) -> int:
    #     # get manual port
    #     self.main_connection.send("DATA START".encode())
    #     response = self.main_connection.recv(1024)
    #     try:
    #         data = response.decode().strip().split()
    #         if data[0] == "OK":
    #             return int(data[1])
    #         else:
    #             return None
    #     except:
    #         return None

       

    # def commands_loop():

    #     pass
    
    # def _start_data():
    #     pass

    # def _start_camera():
    #     pass
    


    # def _keyboard_control(self):
    #     # refine buffer
    #     if 'f' in self.keyboard_buffer and 'b' in self.keyboard_buffer:
    #         self.keyboard_buffer.remove('f')
    #         self.keyboard_buffer.remove('b')
    #     if 'l' in self.keyboard_buffer and 'r' in self.keyboard_buffer:
    #         self.keyboard_buffer.remove('l')
    #         self.keyboard_buffer.remove('r')
        

    # def _stop_keyboard_control(self):
    #     self.view.unbind("<KeyPress>")
    #     self.view.unbind("<KeyRelease>")
    