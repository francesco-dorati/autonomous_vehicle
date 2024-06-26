import socket

MANUAL_CONTROL_FREQ = 10
MANUAL_CONTROL_MS = (1/MANUAL_CONTROL_FREQ)*1000

class ManualController:
    def __init__(self, root, view):
        self.root = root
        self.view = view
        self.main_connection = None
        self.server_hostname = None
        
        self.keyboard_buffer = []
        self.manual_port = None
        self.manual_socket = None

        self.data_port = None
        self.camera_port = None
    
    def start(self, main_connection):
        self.main_connection = main_connection
        self.server_hostname = self.main_connection.get_sock_name()[0]
        m = self._start_manual_control()
            
    def stop(self):
        self._stop_keyboard_control()
    
    def _start_manual_control(self) -> bool:
        # get manual port
        self.main_connection.send("MANUAL START".encode())
        response = self.main_connection.recv(1024)
        try:
            data = response.decode().strip().split()
            if data[0] == "OK":
                self.manual_port = int(data[1])
            else:
                return None
        except:
            return None
        
        self.manual_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.manual_socket.bind((self.server_hostname, self.manual_port))

        self.main_view.focus_set()
        self.main_view.bind("<KeyPress>", self.key_pressed)
        self.main_view.bind("<KeyRelease>", self.key_released)

        self._manual_control()
        return True
    
    def _manual_control(self):
        # refine buffer
        if 'f' in self.keyboard_buffer and 'b' in self.keyboard_buffer:
            self.keyboard_buffer.remove('f')
            self.keyboard_buffer.remove('b')
        if 'l' in self.keyboard_buffer and 'r' in self.keyboard_buffer:
            self.keyboard_buffer.remove('l')
            self.keyboard_buffer.remove('r')
        
        s = "".join(self.keyboard_buffer)
        self.manual_socket.sendto(s.encode(), (self.server_hostname, self.manual_port))
        self.keyboard_buffer.clear()
        if self.manual_socket is not None:
            self.root.after(MANUAL_CONTROL_MS, self._keyboard_control)

    def _stop_manual_control(self):
        self.main_view.unbind("<KeyPress>")
        self.main_view.unbind("<KeyRelease>")
        self.manual_socket.close()
        self.manual_socket = None
        self.main_connection.send("MANUAL STOP".encode())

        
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

       

    def commands_loop():

        pass
    
    def _start_data():
        pass

    def _start_camera():
        pass
    


    def _keyboard_control(self):
        # refine buffer
        if 'f' in self.keyboard_buffer and 'b' in self.keyboard_buffer:
            self.keyboard_buffer.remove('f')
            self.keyboard_buffer.remove('b')
        if 'l' in self.keyboard_buffer and 'r' in self.keyboard_buffer:
            self.keyboard_buffer.remove('l')
            self.keyboard_buffer.remove('r')
        

    def _stop_keyboard_control(self):
        self.main_view.unbind("<KeyPress>")
        self.main_view.unbind("<KeyRelease>")
    
    def _key_pressed(self, event):
        key = event.keysym
        if key == 'w': # FORWARD
            self.keyboard_buffer.append('f')
            self.view.data_frame.forward(True)
        elif key == 's': # BACKWARD
            self.keyboard_buffer.append('b')
            self.view.data_frame.backward(True)
        elif key == 'a':   # LEFT
            self.keyboard_buffer.append('l')
            self.view.data_frame.left(True)
        elif key == 'd':  # RIGHT
            self.keyboard_buffer.append('r')
            self.view.data_frame.right(True)

    def _key_released(self, event):
        key = event.keysym
        if key == 'w':
            self.keyboard_buffer.remove('f')
            self.view.data_frame.forward(False)
        elif key == 's':
            self.keyboard_buffer.remove('b')
            self.view.data_frame.backward(False)
        elif key == 'a':
            self.keyboard_buffer.remove('l')
            self.view.data_frame.left(False)
        elif key == 'd':
            self.keyboard_buffer.remove('r')
            self.view.data_frame.right(False)
