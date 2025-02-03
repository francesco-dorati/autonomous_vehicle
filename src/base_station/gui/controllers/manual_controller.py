import socket
import time


from .camera_controller import CameraReceiver



class ManualController:
    SENDER_DELAY = 100 # ms
    RECEIVER_DELAY = 200 # ms
    def __init__(self, root, view, main_connection):
        self.root = root
        self.view = view
        self.main_connection = main_connection
        self.running = False
        self.manual_hostname = self.main_connection.getpeername()[0]
        self.manual_port = None

        self.camera_receiver = CameraReceiver(self.root, self.view.camera_frame, main_connection)
        
        self.manual_socket = None
        self.vel_buffer = []
        self.x_buffer = []
        self.y_buffer = []

        self.view.start_button.config(command=self.start)
        self.view.stop_button.config(command=self.stop)


    def start(self):
        print("Starting manual controller")
        # start receiver and sender
        self.main_connection.send("M 1\n".encode())
        res = self.main_connection.recv(32)
        res = res.decode().strip().split(' ')
        if res[0] == "OK":
            self.manual_port = int(res[1])
        else:
            return

        self.running = True
        # start servers
        self.manual_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.manual_socket.setblocking(False)

        # set key bindings
        self.view.focus_set()
        self.view.bind("<KeyPress>", self._key_event)
        self.view.bind("<KeyRelease>", self._key_event)
        self.vel_buffer = []
        self.x_buffer = []
        self.y_buffer = []


        # start view
        self.view.start()

        # start loops
        self._sender_loop()
        self._receiver_loop()

    def stop(self):
        self.running = False
        self.view.unbind("<KeyPress>")
        self.view.unbind("<KeyRelease>")
        self.vel_buffer = []
        self.x_buffer = []
        self.y_buffer = []

        self.manual_socket.close()
        self.manual_socket = None
        self.main_connection.send("M 0\n".encode())
        res = self.main_connection.recv(32)
        res = res.decode().strip().split(' ')
        if res[0] != "OK":
            raise Exception("Error stopping manual controller")
        self.view.stop()
        print("Manual Stopped ok")

    def _key_event(self, event): # handles key events
        key = event.keysym
        if key.lower() == 'w': # FORWARD
            if event.type == "2" :
                
                self.view.controls_frame.set('fwd', True)
            elif event.type == "3":
                if 'f' in self.x_buffer:
                    self.x_buffer.remove('f')
                self.view.controls_frame.set('fwd', False)

        elif key.lower() == 's': # BACKWARD
            if event.type == "2":
                if not 'b' in self.x_buffer:
                    self.x_buffer.append('b')
                self.view.controls_frame.set('bwd', True)
            elif event.type == "3":
                if 'b' in self.x_buffer:
                    self.x_buffer.remove('b')
                self.view.controls_frame.set('bwd', False)

        elif key.lower() == 'a':   # LEFT
            if event.type == "2":
                if not 'l' in self.y_buffer:
                    self.y_buffer.append('l')
                self.view.controls_frame.set('left', True)
            elif event.type == "3":
                if 'l' in self.y_buffer:
                    self.y_buffer.remove('l')
                self.view.controls_frame.set('left', False)
                
        elif key.lower() == 'd':  # RIGHT
            if event.type == "2":
                if not 'r' in self.y_buffer:
                    self.y_buffer.append('r')
                self.view.controls_frame.set('right', True)
            elif event.type == "3":
                if 'r' in self.y_buffer:
                    self.y_buffer.remove('r')
                self.view.controls_frame.set('right', False)
        
        elif key == 'Shift_L':
            if event.type == "2":
                if not 's' in self.vel_buffer:
                    self.vel_buffer.append('s')
                self.view.controls_frame.set('slow', True)
            elif event.type == "3":
                if 's' in self.vel_buffer:
                    self.vel_buffer.remove('s')
                self.view.controls_frame.set('slow', False)
        
        elif key == 'space':
            if event.type == "2":
                if not 'b' in self.vel_buffer:
                    self.vel_buffer.append('b')
                self.view.controls_frame.set('fast', True)
            elif event.type == "3":
                if 'b' in self.vel_buffer:
                    self.vel_buffer.remove('b')
                self.view.controls_frame.set('fast', False)

        # print(key, self.boost_buffer, self.keyboard_buffer)

    def _sender_loop(self): # sends commands to the rpi
        print(f"loop: {self.running}")
        if self.running:
            vel, x, y = self._transform_buffers()

            command = f"{vel} {x} {y}"
            print(f"Keyboard buffer: ", self.vel_buffer, self.x_buffer, self.y_buffer)
            print(f"Sent: <{command}> to ", self.manual_hostname, ":", self.manual_port)
            
            try:
                self.manual_socket.sendto(command.encode(), (self.manual_hostname, self.manual_port))
            except:
                print("ERROR, stopping")
                self.stop()
                return

            self.root.after(self.SENDER_DELAY, self._sender_loop)

    def _receiver_loop(self): # receives data from the rpi
        if self.running:
            try:
                data, _ = self.manual_socket.recvfrom(64)
                odometry, distances = self._parse_data(data.decode())
                print("Received:", odometry, distances)
                self.view.odometry_frame.update(odometry[0], odometry[2:])
                self.view.sensors_frame.update(distances)
            except BlockingIOError:
                pass

            
            # except Exception as e:
            #     print("ERROR, stopping", e)
            #     self.stop()
            #     return

            self.root.after(self.RECEIVER_DELAY, self._receiver_loop)

    def _parse_data(self, data):
        odometry = [0.0, 0.0, 0.0, 0.0, 0.0]
        distances = [0.0, 0.0, 0.0, 0.0]
        data = data.strip().split('\n')
        for line in data:
            data = line.strip().split(' ')
            if data[0] == 'E':
                odometry = [float(data[1]), float(data[2]), float(data[3]), float(data[4]), float(data[5])]
            elif data[0] == 'D':
                distances = [float(data[1]), float(data[2]), float(data[3]), float(data[4])]
        return odometry, distances

    def _transform_buffers(self):
        if 'f' in self.x_buffer and 'b' in self.x_buffer:
            self.x_buffer.remove('f')
            self.x_buffer.remove('b')
        if 'l' in self.y_buffer and 'r' in self.y_buffer:
            self.y_buffer.remove('l')
            self.y_buffer.remove('r')
        if 'b' in self.vel_buffer and 's' in self.vel_buffer:
            self.vel_buffer.remove('b')
            self.vel_buffer.remove('s')
        x = 0
        for c in self.x_buffer:
            if c == 'f':
                x += 1
            elif c== 'b':
                x += -1
        y = 0
        for c in self.y_buffer:
            if c == 'l':
                y += 1
            elif c== 'r':
                y += -1
        vel = 0
        for c in self.vel_buffer:
            if c == 'b':
                vel += 1
            elif c== 's':
                vel += -1

        return vel, x, y



# class ControlsSender:
#     def __init__(self, root, view, main_connection):
#         self.root = root
#         self.view = view
#         self.main_connection = main_connection
#         self.server_hostname = self.main_connection.getpeername()[0]

#         self.controls_port = None
#         self.controls_socket = None
#         self.is_running = False
        
#         self.keyboard_buffer = []


#     def start(self):
#         self.main_connection.send("MANUAL START".encode())
#         response = self.main_connection.recv(1024)
#         try:
#             data = response.decode().strip().split()
#             if data[0] == "OK":
#                 self.controls_port = int(data[1])
#             else:
#                 return None
#         except:
#             return None
        
#         try:
#             self.controls_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
#         except:
#             return None
        
#         # KEYBINDINGS
#         self.view.focus_set()
#         self.view.bind("<KeyPress>", self._key_pressed)
#         self.view.bind("<KeyRelease>", self._key_released)

#         self.is_running = True
#         self.view.enable()
#         self._sender_loop()
#         return True
    
#     def _sender_loop(self):
#         # refine buffer
#         if self.is_running:
#             if 'f' in self.keyboard_buffer and 'b' in self.keyboard_buffer:
#                 self.keyboard_buffer.remove('f')
#                 self.keyboard_buffer.remove('b')
#             if 'l' in self.keyboard_buffer and 'r' in self.keyboard_buffer:
#                 self.keyboard_buffer.remove('l')
#                 self.keyboard_buffer.remove('r')
            
#             s = "".join(self.keyboard_buffer)
#             # print("Sent:", s, " to ", self.server_hostname, ":", self.manual_port)
#             try:
#                 self.controls_socket.sendto(s.encode(), (self.server_hostname, self.controls_port))
#             except:
#                 print("ERROR, stopping")
#                 self.stop()
#                 return

#             self.root.after(MANUAL_CONTROL_MS, self._sender_loop)
    
#     def stop(self):
#         self.is_running = False
#         self.view.unbind("<KeyPress>")
#         self.view.unbind("<KeyRelease>")
#         self.controls_socket.close()
#         self.controls_socket = None
#         self.main_connection.send("M 0".encode())
#         self.view.disable()
  


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



# class DataReceiver:
#     def __init__(self, root, view, main_connection):
#         self.root = root
#         self.view = view
#         self.main_connection = main_connection
#         self.server_hostname = self.main_connection.getpeername()[0]

#         self.data_port = None
#         self.data_socket = None
#         self.is_running = False

#     def start(self):
#         self.main_connection.send("DATA START".encode())
#         response = self.main_connection.recv(1024)
#         try:
#             data = response.decode().strip().split()
#             if data[0] == "OK":
#                 self.data_port = int(data[1])
#             else:
#                 return None
#         except:
#             return None
        
#         # SEND DATAT RATE
        
#         try:
#             self.data_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
#             self.data_socket.sendto("START".encode(), (self.server_hostname, self.data_port))
#         except:
#             return None
        
#         self.is_running = True
#         self._receiver_loop()
#         return True
    
#     def _receiver_loop(self):
#         if self.is_running:
#             try:
#                 ready = select.select([self.data_socket], [], [], 0.1)
#                 if ready[0]:
#                     data, _ = self.data_socket.recvfrom(1024)
#                     vel, pos = self._parse_data(data.decode())
#                     self.view.update(vel, pos)

#             except:
#                 self.stop()
#                 return
#             self.root.after(100, self._receiver_loop)

#     def _parse_data(self, data):
#         data = data.strip().split()
#         vel = [float(data[0]), float(data[1])]
#         pos = [float(data[2]), float(data[3]), float(data[4])]
#         return vel, pos

#     def stop(self):
#         self.is_running = False
#         self.data_socket.close()
#         self.data_socket = None
#         self.main_connection.send("DATA STOP".encode())
#         self.view.disable()


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
    