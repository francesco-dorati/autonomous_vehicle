import time
import socket
import serial
import cv2
import pickle
import os
import smbus

from rp2040 import RP2040
from nano import NANO
from camera import CameraTransmitter

class Main:
    pass

class MainServer:
    def __init__(self):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.bind((HOST, MAIN_PORT))
        self.socket.listen(1)
        self.socket.setblocking(False)

        self.connection = None
        self.connection_addr = None

    def check_connection(self):
        try:
            self.connection, self.connection_addr = self.socket.accept()
            self.connection.setblocking(False)
            return True

        except BlockingIOError:
            return False
        
    def get_command(self):
        try:
            data = self.connection.recv(32)
            return data
        except BlockingIOError:
            return None
        
    def send(self, data: str):
        self.connection.send(data.encode())
    
    def close_connection(self):
        self.connection = None
        self.connection_addr = None
        self.connection.close()

class ManualReceiver:
    def __init__(self):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind((HOST, M_RECEIVER_PORT))
        self.socket.setblocking(False)
    
    def get_command(self):
        data, _ = self.socket.recv(32)
        d = data.decode().split(' ')
        speed = int(d[0])
        direction = d[1]
        return speed, direction
    
    def close(self):
        self.socket.close()
        

class ManualTransmitter:
    def __init__(self):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind((HOST, M_TRANSMITTER_PORT))
        self.socket.setblocking(False)
        self.client = None

    def send_data(self, encoder_odometry, obstacle_distance):
        if self.client == None:
            _, addr = self.socket.recvfrom(5)
            self.client = (addr[0], int(addr[1]))
        message = f'P {encoder_odometry.vx} {encoder_odometry.vt} {encoder_odometry.x} {encoder_odometry.y} {encoder_odometry.t}'
        message += f' D {obstacle_distance[0]} {obstacle_distance[1]} {obstacle_distance[2]} {obstacle_distance[3]}'
        self.socket.sendto(message.encode(), self.client)

    def close(self):
        self.socket.close()


#CONSTANTS
class state(enumerate):
    NOT_CONNECTED: -1
    IDLE: 0
    MANUAL: 1

class Odometry:
    def __init__(self):
        self.vx = 0.0
        self.vt = 0.0
        self.x = 0.0
        self.y = 0.0
        self.t = 0.0

# FREQUENCIES
MANUAL_FREQ = 50 # Hz
MANUAL_TRANSMITTER_FREQ = 5 # Hz

# TIME 
NOT_CONNECTED_DELAY = 1 # s
IDLE_DELAY = 0.1 # s
MANUAL_DELAY = 1/MANUAL_FREQ # s 
MANUAL_TRANSMITTER_DELAY = 1/MANUAL_TRANSMITTER_FREQ # s
CAMERA_TRANSMITTER_DELAY = 0.033 # s

# HOST
HOST = ''
MAIN_PORT = 5500
M_RECEIVER_PORT = 5501
M_TRANSMITTER_PORT = 5502
CAMERA_PORT = 5503
# BATTERY
BATTERY_CHECK_INTERVAL = 10 # s
BATTERY_MEDIUM = 11.3 # V
BATTERY_LOW = 10.5 # V
BATTERY_CRITICAL = 9.8 # V
# MANUAL
MANUAL_STD_POW = 100
MANUAL_BOOST_POW = 150
MANUAL_SLOW_POW = 50



# VARIABLES
# state
main_state = state.NOT_CONNECTED

# servers
main_server = MainServer()

# external controllers
rp2040 = RP2040()
nano = NANO()

# data
data_updated = False
battery_voltage = 0.0
encoder_odometry = None
obstacle_distance = None

# battery
last_battery_check = 0

# manual
manual_receiver = None
manual_transmitter = None
last_manual_transmitted = 0
distance_sensing = False

# camera
camera_transmitter = None
last_camera_transmitted = 0


def main_loop():
    # start battery sensing
    rp2040.set_battery(True)

    while True:
        t_start = time.time()
        data_updated = False

        #1 check battery level 
        if (time.time() - last_battery_check) >= BATTERY_CHECK_INTERVAL: 
            check_battery()

        # 2 MAIN CONNECTION HANDLING
        # 2.a wait for connection
        if main_state == state.NOT_CONNECTED:
            ok = main_server.check_connection()
            if ok:
                main_state = state.IDLE
            else:
                continue
        # 2.b handle main request
        else:
            try:
                d = main_server.get_command()
                handle_main_request(d)
            except ConnectionResetError:
                main_server.close()
                main_state = state.NOT_CONNECTED
                continue
            

        # 3 LOOP
        # 3.a MANUAL LOOP
        if main_state == state.MANUAL: 
            # 3.a.1 request data
            update_data()

            # 3.a.2 handle manual command
            speed, direction = manual_receiver.get_command()

            # 3.a.3 calculate motor speed
            pow_l = 0
            pow_r = 0
            if distance_sensing:
                # calculate speed based on distance
                pass

            else:

                if speed == -1:
                    pow_l = MANUAL_SLOW_POW
                    pow_r = MANUAL_SLOW_POW
                    
                elif speed == 0:
                    pow_l = MANUAL_STD_POW
                    pow_r = MANUAL_STD_POW

                elif speed == 1:
                    pow_l = MANUAL_BOOST_POW
                    pow_r = MANUAL_BOOST_POW
            
        
            # 3.a.4 send motor commands
            nano.send_power(pow_l, pow_r)

            # 3.a.5 transmit data
            if (time.time() - last_manual_transmitted) >= MANUAL_TRANSMITTER_DELAY:
                last_manual_transmitted = time.time()
                manual_transmitter.send_data(encoder_odometry, obstacle_distance)

        # 4 update camera
        if camera_transmitter != None and (time.time() - last_camera_transmitted) >= CAMERA_TRANSMITTER_DELAY:
            last_camera_transmitted = time.time()
            camera_transmitter.send_frame()

        # 5 set delay
        dt = time.time() - t_start
        if main_state == state.NOT_CONNECTED and dt < NOT_CONNECTED_DELAY:
            time.sleep(NOT_CONNECTED_DELAY-dt)
        elif main_state == state.IDLE and dt < IDLE_DELAY:
            time.sleep(IDLE_DELAY - dt)
        elif main_state == state.MANUAL and dt < MANUAL_DELAY:
            time.sleep(MANUAL_DELAY - dt)

def update_data():
    if not data_updated:
        battery_voltage, encoder_odometry, obstacle_distance = rp2040.request_data()
        data_updated = True

def check_battery():
    update_data()
    last_battery_check = time.time()
    if (battery_voltage <= BATTERY_CRITICAL):
        shutdown()
    
def handle_main_request(data):
    d = data.decode().split(' ')

    if d[0] == 'P': # PING
        update_data()
        main_server.send('P ' + str(battery_voltage))

    elif d[0] == 'M': # MANUAL
        if d[1] == '1': # start manual mode
            if main_state != state.IDLE:
                main_server.send(b'KO')
                return
            start_manual_mode()
            main_server.send(f'OK {M_RECEIVER_PORT} {M_TRANSMITTER_PORT}'.encode())

        elif d[1] == b'0': # stop manual mode
            if main_state != state.MANUAL:
                main_server.send(b'KO')
                return
            stop_manual_mode()
            main_server.send(b'OK')
        
        elif d[1] == b'S':
            if main_state != state.MANUAL:
                main_server.send(b'KO')
                return

            # enable/disable sensors
            for e in d[2:]:
                if e[0] == 'E':
                    # encoders
                    set_encoder_odometry(int(e[1]))
                    pass
                elif e[0] == 'D':
                    # distance
                    set_distance_sensing(int(e[1]))
                    pass
                else:
                    main_server.send(b'KO')
                    return
                
    elif d[0] == 'C': # CAMERA
        if d[1] == '1':
            camera_transmitter = CameraTransmitter()
            main_server.send(f'OK {CAMERA_PORT}'.encode())
        elif d[1] == '0':
            camera_transmitter.close()
            camera_transmitter = None
            main_server.send(b'OK')

    elif d[0] == 'E': # STOP
        main_server.send(b'OK')
        close_all()
        main_state = state.NOT_CONNECTED

    elif d[0] == 'S': # SHUTDOWN
        main_server.send(b'OK')
        shutdown()
        

def start_manual_mode():
    manual_receiver = ManualReceiver()
    manual_transmitter = ManualTransmitter()

    main_state = state.MANUAL

def stop_manual_mode():
    # stop distance sensing and encoder
    manual_receiver.close()
    manual_transmitter.close()
    manual_receiver = None
    manual_transmitter = None
    rp2040.set_encoder(False)
    rp2040.set_distance(False)
    nano.send_power(0, 0)
    main_state = state.IDLE
    pass

def set_distance_sensing(n):
    distance_sensing = True if n == 2 else False
    rp2040.set_distance(False if n == 0 else True)

def set_encoder_odometry(n):
    rp2040.set_encoder(False if n == 0 else True)

def close_all():
    if camera_transmitter != None:
        camera_transmitter.close()
    if main_state == state.MANUAL:
        stop_manual_mode()
    main_server.close_connection()
    RP2040.set_battery(False)
    RP2040.set_encoder(False)
    RP2040.set_distance(False)
    nano.send_power(0, 0)
    
def shutdown():
    close_all()
    os.system("sudo shutdown now")

if __name__ == '__main__':
    main_loop()