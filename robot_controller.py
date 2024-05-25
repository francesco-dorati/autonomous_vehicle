import time
import socket
import serial
import keyboard
import queue
import threading
from enum import Enum

SERIAL_PORT = '/dev/ttyUSB0'
SERIAL_RATE = 115200

AUTO_SOCKET_PORT = 5500
MANUAL_SOCKET_PORT = 5501

# BASE_LOOP_FREQ = 50
# UPDATE_TAO = (1/BASE_LOOP_FREQ)

AUTO_LOOP_FREQ = 50 # Hz
AUTO_TAO = (1/AUTO_LOOP_FREQ) # s
AUTO_LIN_VEL = 20 # cm/s
AUTO_ANG_VEL = 60 # deg/s
AUTO_LIN_ERROR = 2 # cm
AUTO_ANG_ERROR = 5 # deg

MANUAL_LOOP_FREQ = 10 # Hz
MANUAL_TAO = (1/MANUAL_LOOP_FREQ) # s
MANUAL_LIN_VEL = 15 # cm/s
MANUAL_ANG_VEL = 50 # deg/s


class Mode(Enum):
    WAIT_CONNECTION = 0
    AUTO = 1
    MANUAL = 2
    
class TCPServer(threading.Thread):
    def __init__(self, hostname, port):
        super().__init__(daemon=True)
        self.hostname = hostname
        self.port = port
        self.queue = queue.Queue()
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.running = True
        self.connected = False
    
    def run(self):
        self.socket.bind((self.hostname, self.port))
        self.server_socket.listen(1)
        print(f"TCP server listening on {self.hostname}:{self.port}")
        while self.running:
            try:
                self.connection, addr = self.socket.accept()
                print("Established a connection with", addr)
                threading.Thread(target=self.handle_client, daemon=True).start()
            except socket.timeout:
                continue

    def handle_client(self):
        self.connected = True
        while self.running:
            try:
                data = self.connection.recv(1024)
                if data:
                    self.queue.put(data.decode())
            except socket.error:
                break

        self.end_connection()

    def send(self, data):
        if not self.connected:
            raise Exception("No client connected")
        self.connection.send(data.encode())

    def end_connection(self): 
        self.connection.close()
        self.connected = False

    def stop(self):
        self.running = False
        self.socket.close()

class UDPServer(threading.Thread):
    def __init__(self, host, port):
        super().__init__(daemon=True)
        self.host = host
        self.port = port
        self.queue = queue.Queue()
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.running = True
        self.connected = False
    
    def run(self):
        self.socket.bind((self.host, self.port))
        print(f"UDP server listening on {self.hostname}:{self.port}")
        while self.running:
            try:
                data, addr = self.server_socket.recvfrom(1024)
                self.queue.put(data.decode())
                self.connected = True
            except socket.error:
                continue

    def end_connection(self):
        self.connected = False

    def stop(self):
        self.running = False
        self.server_socket.close()

class SerialClient:
    def __init__(self, port, baud_rate):
        print("Starting serial...")
        self.serial = serial.Serial(port, baud_rate, timeout=0)
        time.sleep(2)
        if not self.serial.isOpen():
            print("Serial port is closed. Exiting..")
            exit(1)
        else:
            print("Successfully connected.")

    def send(self, lin_vel, ang_vel):
        self.serial.write(f"{lin_vel} {ang_vel}".encode())

    def read(self):
        return self.serial.readline().decode()
    
class AutonomousController:
    def __init__(self, socket, serial):
        self.socket = socket
        self.serial = serial
        self.current_state = (.0, .0, .0)
        self.movements_queue = queue.Queue()
        self.goal_position = (.0, .0, .0)

    def loop(self):
        while True:
            # read auto queue
            t_start = time.time()

            # if command in socket buffer
            if not self.socket.queue.empty():
                command = self.socket.queue.get()
                if command == "EXIT":
                    self._reset()
                    return

                self._add_command(command)

            # receive odometry data
            data = self.serial.read()
            if data:
                dx, dy, dt = map(float, data.split())
                self._update_state(dx, dy, dt)

            # calculate goal velocity
            lin_vel, ang_vel = self._calculate_goal_velocity()

            # send to serial
            self.serial.send(lin_vel, ang_vel)
            self.socket.send(f"{self.current_state[0]} {self.current_state[1]} {self.current_state[2]}")

            dt = time.time() - t_start
            if AUTO_TAO < dt:
                time.sleep(AUTO_TAO - dt)
            

    def _add_command(self, command):
        try:
            c = command.split()
            if c[0] == "mv":
                self.commands_queue.put((int(c[1]), .0, .0))
                return
            
            elif c[0] == "rot":
                self.commands_queue.put((.0, .0, int(c[1])))
                return
        except:
            raise Exception(f"Invalid command: '{command}'")
    
        
    def _update_state(self, dx, dy, dt):
        self.current_state[0] += dx
        self.current_state[1] += dy
        self.current_state[2] += dt

    def _calculate_goal_velocity(self):
        dx = self.goal_position[0] - self.current_state[0]
        dy = self.goal_position[1] - self.current_state[1]
        dt = self.goal_position[2] - self.current_state[2]
        while abs(dx) < AUTO_LIN_ERROR and abs(y) < AUTO_LIN_ERROR and abs(dt) < AUTO_ANG_ERROR:
            # GOAL REACHED
            # check new command
            if self.movements_queue.empty():
                return (.0, .0)
            
            dx, dy, dt = self.movements_queue.get()
            self.goal_position[0] += dx
            self.goal_position[1] += dy
            self.goal_position[2] += dt

        # calculate velocity
        if dx > 0:
            lin_vel = AUTO_LIN_VEL
        elif dx < 0:
            lin_vel = -AUTO_LIN_VEL
        
        if dt > 0:
            ang_vel = AUTO_ANG_VEL
        elif dt < 0:
            ang_vel = -AUTO_ANG_VEL

        return (lin_vel, ang_vel)

    def _reset(self):
        self.current_state = (.0, .0, .0)
        self.movements_queue = queue.Queue()
        

class ManualController:
    def __init__(self, socket, serial):
        self.socket = socket
        self.serial = serial

    def loop(self):
        while True:
            t_start = time.time()
            lin_vel, ang_vel = (.0, .0)

            if not socket.queue.empty():
                keyboard_buffer = socket.queue.get()
                if keyboard_buffer == "EXIT":
                    return
                if "f" in keyboard_buffer:
                    lin_vel += MANUAL_LIN_VEL
                if "b" in keyboard_buffer:
                    lin_vel -= MANUAL_LIN_VEL
                if "l" in keyboard_buffer:
                    ang_vel += MANUAL_ANG_VEL
                if "r" in keyboard_buffer:
                    ang_vel -= MANUAL_ANG_VEL

            serial.send(lin_vel, ang_vel)

            dt = time.time() - t_start
            if MANUAL_TAO < dt:
                time.sleep(MANUAL_TAO - dt)

    def end_connection(self):
        self.socket.end_connection()

def main():
    # Start Sockets
    auto_socket = TCPServer(socket.gethostname(), AUTO_SOCKET_PORT)
    manual_socket = UDPServer(socket.gethostname(), MANUAL_SOCKET_PORT)
    serial = SerialClient(SERIAL_PORT, SERIAL_RATE)

    auto_socket.start()
    manual_socket.start()

    mode == Mode.WAIT_CONNECTION
    while True:
        if mode == Mode.WAIT_CONNECTION:
            if auto_socket.connected and manual_socket.connected:
                auto_socket.end_connection()
                manual_socket.end_connection()
            elif auto_socket.connected:
                mode = Mode.AUTO
            elif manual_socket.connected:
                mode = Mode.MANUAL

        elif mode == Mode.AUTO:
            auto_controller = AutonomousController(auto_socket, serial)
            auto_controller.loop()
            auto_socket.end_connection()
            mode = Mode.WAIT_CONNECTION
            continue

        elif mode == Mode.MANUAL:
            manual_controller = ManualController(manual_socket, serial)
            manual_controller.loop()
            manual_socket.end_connection()
            mode = Mode.WAIT_CONNECTION
            continue

        time.sleep(0.1)

    
if __name__ == "__main__":
    main()
