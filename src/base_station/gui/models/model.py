import socket
from typing import Tuple
import threading
import time

class Model:
    def __init__(self):
        self.ROBOT_ADDRESS = '172.20.10.3'
        self.MAIN_PORT = 5500
        self.main_connection = None

        self.battery_V: float = None    # V
        self.ping_time: int = None      # ms
        self.global_map_name: str = None

        self.control_type: str = None

        self.manual_thread = None
        self.manual_connection = None
        self.manual_control_type: str = None
        self.keyboard_buffer = {'x': 0, 'y': 0}
        self.joypad = None
        self.joypad_buffer = {'x': 0.0, 'y': 0.0}

    def connect(self) -> bool:
        self.main_connection = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.main_connection.settimeout(1)
        try:
            self.main_connection.connect((self.ROBOT_ADDRESS, self.MAIN_PORT))
        except (socket.timeout, ConnectionRefusedError):
            self.main_connection = None
            return False
        return True
    
    def disconnect(self):
        if self.main_connection:
            # SEND COMMAND
            self.main_connection.close()
            self.main_connection = None


    def ping(self) -> Tuple[bool, bool, bool]:
        """ TODO Pings the server to check if it is still connected 
            Returns:
                bool: ping successful
                bool: update control
                bool: update map
        """
        try:
            self.main_connection.send("SYS PNG\n".encode())
            response = self.main_connection.recv(32)
            data = response.decode().strip().split()
            if data[0] == "OK":
                return True
        except Exception as e:
            print("PING failed, error: ", e)
            return False
        return True

    def new_global_map(self, name):
        # send to serial
        self.global_map_name = name

    def set_control(self, control_type: str):
        self.control_type = control_type

        if control_type == "manual":
            self.manual_control_type = "keyboard"
            self.keyboard_buffer = {'x': 0, 'y': 0}
            self.joypad_buffer = {'x': 0, 'y': 0}
            try: # start manual receiver
                self.main_connection.send("CTL MAN\n".encode())
                response = self.main_connection.recv(32).decode().strip()
                split = response.split(" ")
                if split[0] == "OK" and int(split[1]):
                    self.start_manual_connection(int(split[1]))

            except Exception as e:
                print("error: ", e)

        elif control_type == "off":
            if self.manual_connection:
                self.stop_manual_connection()

    def start_manual_connection(self, port: int):
        self.manual_connection = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.manual_connection.settimeout(1)
        try:
            self.manual_connection.connect((self.ROBOT_ADDRESS, port))
        except (socket.timeout, ConnectionRefusedError):
            self.manual_connection = None
            return
        self.manual_thread = threading.Thread(target=self.manual_transmitter_worker, daemon=True)
        self.manual_thread.start()
    
    def stop_manual_connection(self):
        if self.manual_connection:
            self.manual_connection.close()
        self.manual_connection = None
        self.manual_thread.join()
        self.manual_thread = None
    
    def manual_transmitter_worker(self):
        while self.manual_connection:
            data = ""
            if self.manual_control_type == "keyboard":
                data = f"KEY {self.keyboard_buffer['x']} {self.keyboard_buffer['y']}\n"
            elif self.manual_control_type == "joypad":
                data = f"JOY {self.joypad_buffer['x']:.2f} {self.joypad_buffer['y']:.2f}\n"
            else:
                break

            try:
                self.manual_connection.send(data.encode())
            except Exception as e:
                print("error: ", e)
                break

            time.sleep(0.1)



    def __str__(self):
        return str(self.data)