import socket
import select
import pickle
import cv2
from PIL import Image
import numpy as np

from .camera_controller import CameraReceiver



class ManualController:
    SENDER_DELAY = 0.1
    RECEIVER_DELAY = 0.2
    def __init__(self, root, view, main_connection):
        self.root = root
        self.view = view
        self.main_connection = main_connection
        self.running = False
        self.server_hostname = self.main_connection.getpeername()[0]
        self.boost_buffer = 0
        self.keyboard_buffer = []

        self.commands_socket = None
        self.data_socket = None

        # self.controls_sender = ControlsSender(self.root, self.view.controls_frame, main_connection)
        # self.data_receiver = DataReceiver(self.root, self.view.data_frame, main_connection)
        # self.camera_receiver = CameraReceiver(self.root, self.view.camera_frame, main_connection)

        self.view.start_button.config(command=self.start)
        self.view.stop_button.config(command=self.stop)

        self.view.camera_frame.start_button.config(command=self.start_camera)
        self.view.camera_frame.stop_button.config(command=self.stop_camera)


    def start(self):
        # start receiver and sender
        self.main_connection.send("M 1".encode())
        res = self.main_connection.recv(32)
        res = res.decode().strip().split(' ')
        if res[0] == "OK":
            self.manual_port = int(res[1])
        else:
            return

        # start servers
        self.manual_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.manual_socket.setblocking(False)

        # set key bindings
        self.view.focus_set()
        self.view.bind("<KeyPress>", self._key_event)
        self.view.bind("<KeyRelease>", self._key_event)

        # start view
        self.view.start()
        runnning = True

        # start loops
        self._sender_loop()
        self._receiver_loop()

    def stop(self):
        self.running = False
        self.view.unbind("<KeyPress>")
        self.view.unbind("<KeyRelease>")
        self.manual_socket.close()
        self.manual_socket = None
        self.main_connection.send("M 0".encode())
        self.view.stop()

    def _key_event(self, event):
        key = event.keysym
        if key == 'w': # FORWARD
            if event.type == "2":
                if not 'f' in self.keyboard_buffer:
                    self.keyboard_buffer.append('f')
                self.view.set('fwd', True)
            elif event.type == "3":
                if 'f' in self.keyboard_buffer:
                    self.keyboard_buffer.remove('f')
                self.view.set('fwd', False)

        elif key == 's': # BACKWARD
            if event.type == "2":
                if not 'b' in self.keyboard_buffer:
                    self.keyboard_buffer.append('b')
                self.view.set('bwd', True)
            elif event.type == "3":
                if 'b' in self.keyboard_buffer:
                    self.keyboard_buffer.remove('b')
                self.view.set('bwd', False)

        elif key == 'a':   # LEFT
            if event.type == "2":
                if not 'l' in self.keyboard_buffer:
                    self.keyboard_buffer.append('l')
                self.view.set('left', True)
            elif event.type == "3":
                if 'l' in self.keyboard_buffer:
                    self.keyboard_buffer.remove('l')
                self.view.set('left', False)
                
        elif key == 'd':  # RIGHT
            if event.type == "2":
                if not 'r' in self.keyboard_buffer:
                    self.keyboard_buffer.append('r')
                self.view.set('right', True)
            elif event.type == "3":
                if 'r' in self.keyboard_buffer:
                    self.keyboard_buffer.remove('r')
                self.view.set('right', False)
        
        elif key == 'shift':
            if event.type == "2":
                if not 's' in self.boost_buffer:
                    self.boost_buffer.append('s')
                self.view.set('slow', True)
            elif event.type == "3":
                if 's' in self.boost_buffer:
                    self.boost_buffer.remove('s')
                self.view.set('slow', False)
        
        elif key == 'space':
            if event.type == "2":
                if not 'b' in self.boost_buffer:
                    self.boost_buffer.append('b')
                self.view.set('fast', True)
            elif event.type == "3":
                if 'b' in self.boost_buffer:
                    self.boost_buffer.remove('b')
                self.view.set('fast', False)

    def _sender_loop(self):
        if self.running:
            if 'f' in self.keyboard_buffer and 'b' in self.keyboard_buffer:
                self.keyboard_buffer.remove('f')
                self.keyboard_buffer.remove('b')
            if 'l' in self.keyboard_buffer and 'r' in self.keyboard_buffer:
                self.keyboard_buffer.remove('l')
                self.keyboard_buffer.remove('r')
            if 'b' in self.boost_buffer and 's' in self.boost_buffer:
                self.boost_buffer.remove('b')
                self.boost_buffer.remove('s')

            b = 0 if (len(self.boost_buffer) == 0) else (1 if self.boost_buffer[0] == 'b' else -1)
            s = "".join(self.keyboard_buffer)

            # print("Sent:", s, " to ", self.server_hostname, ":", self.manual_port)
            try:
                command = f"{b} {s}"
                self.controls_socket.sendto(command.encode(), (self.server_hostname, self.controls_port))
            except:
                print("ERROR, stopping")
                self.stop()
                return

            self.root.after(self.SENDER_DELAY, self._sender_loop)

    def _receiver_loop(self):
        if self.running:
            pass
            # try:
            #     data, _ = self.manual_socket.recvfrom(32)
            #     vel, pos = self._parse_data(data.decode())
            #     self.view.update(vel, pos)
            # except:
            #     self.stop()
            #     return
            # self.root.after(100, self._receiver_loop)


    
    def start_controls(self):
        if self.controls_sender.is_running:
            return
        ok = self.controls_sender.start()
        if not ok:
            self.view.controls_frame.disable()
    def stop_controls(self):
        if not self.controls_sender.is_running:
            return
        self.controls_sender.stop()
    
    def start_camera(self):
        if self.camera_receiver.is_running:
            return
        ok = self.camera_receiver.start()
        if not ok:
            self.view.camera_frame.disable()
    def stop_camera(self):
        if not self.camera_receiver.is_running:
            return
        self.camera_receiver.stop()

    def start_data(self):
        pass
    def stop_data(self):
        pass
        
    def stop(self):
        self.stop_camera()
        self.stop_controls()
        self.start_data()

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
  


    # def _key_released(self, event):
    #     key = event.keysym
    #     if key == 'w':
    #         if 'f' in self.keyboard_buffer:
    #             self.keyboard_buffer.remove('f')
    #         self.view.forward(False)
    #     elif key == 's':
    #         if 'b' in self.keyboard_buffer:
    #             self.keyboard_buffer.remove('b')
    #         self.view.backward(False)
    #     elif key == 'a':
    #         if 'l' in self.keyboard_buffer:
    #             self.keyboard_buffer.remove('l')
    #         self.view.left(False)
    #     elif key == 'd':
    #         if 'r' in self.keyboard_buffer:
    #             self.keyboard_buffer.remove('r')
    #         self.view.right(False)



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
    