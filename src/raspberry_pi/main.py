import time
import socket
import cv2
import pickle
import os
from enum import Enum


from rp2040 import RP2040
from nano import NANO
from manual import ManualController
from camera import CameraTransmitter

class Main:
    # CONST
    # host
    HOST = '172.20.10.3'
    MAIN_PORT = 5500
    M_RECEIVER_PORT = 5501
    M_TRANSMITTER_PORT = 5502
    CAMERA_PORT = 5503

    # delays
    BATTERY_CHECK_INTERVAL = 3 # s
    MAIN_SERVER_INTERVAL = 0.4 # s
    MANUAL_LOOP_DELAY = 1/50 # s

    # frequencies
    MANUAL_FREQ = 50 # Hz
    MANUAL_TRANSMITTER_FREQ = 5 # Hz

    # time 
    NOT_CONNECTED_DELAY = 1 # s
    IDLE_DELAY = 0.1 # s
    MANUAL_DELAY = 1/MANUAL_FREQ # s 
    MANUAL_TRANSMITTER_DELAY = 1/MANUAL_TRANSMITTER_FREQ # s
    CAMERA_TRANSMITTER_DELAY = 0.033 # s
    # battery

    #  manual
    MANUAL_STD_POW = 100
    MANUAL_BOOST_POW = 150
    MANUAL_SLOW_POW = 50

    class Mode(Enum):
        NOT_CONNECTED = -1
        IDLE = 0
        MANUAL = 1
    
    def __init__(self):
        self.mode = self.Mode.NOT_CONNECTED

        self.main_server = MainServer(self.HOST, self.MAIN_PORT) # handles main connection to the user

        self.rp2040 = RP2040() # handles communication with the rp2040 (SENSOR DATA)
        self.nano = NANO() # handles communication with the nano (ACTION EXECUTION)

        self.last_battery_check = 0
        self.last_main_server_check = 0
        self.last_camera_transmitted = 0
        
        self.manual_controller = None
        self.camera_transmitter = None
        
        self.distance_sensing = False

    def loop(self):
        # start battery sensing
        self.rp2040.set_battery(True)

        while True:
            t_start = time.time()
            self.rp2040.updated = False

            #1 BATTERY CHECK
            if (time.time() - self.last_battery_check) >= self.BATTERY_CHECK_INTERVAL: 
                self.last_battery_check = time.time()
                self.rp2040.request_data()
                if (self.rp2040.battery.is_critical()):
                    self.shutdown()

            # 2 MAIN CONNECTION HANDLING
            if (time.time() - self.last_main_server_check) >= self.MAIN_SERVER_INTERVAL:
                self.last_main_server_check = time.time()
                if self.mode == self.Mode.NOT_CONNECTED: # not connected
                    ok = self.main_server.check_connection()
                    if ok:
                        self.mode = self.Mode.IDLE
                    continue
                
                try: # receive request
                    req = self.main_server.get_request()
                    if d != None:
                        self.handle_request(req)

                except ConnectionResetError:
                    self.main_server.close()
                    self.mode = self.Mode.NOT_CONNECTED
                    continue


            # 3 CONTROL
            if self.mode == self.Mode.MANUAL:
                self.manual_controller.compute()

            # 4 update camera
            if self.camera_transmitter != None and (time.time() - self.last_camera_transmitted) >= self.CAMERA_TRANSMITTER_DELAY:
                self.last_camera_transmitted = time.time()
                camera_transmitter.send_frame()

            # 5 set delay
            dt = time.time() - t_start
            if (self.mode == self.Mode.NOT_CONNECTED or self.mode == self.Mode.IDLE) and dt < self.MAIN_SERVER_INTERVAL:
                time.sleep(self.MAIN_SERVER_INTERVAL-dt)
            elif self.mode == self.Mode.MANUAL and dt < self.MANUAL_DELAY:
                time.sleep(self.MANUAL_DELAY - dt)


    
        
    def handle_request(self, data):
        d = data.decode().split(' ')

        if d[0] == 'P': # PING
            self.update_rp2040()
            self.main_server.send('P ' + str(self.rp2040.battery.voltage))

        elif d[0] == 'M': # MANUAL
            if d[1] == '1': # start manual mode
                if self.mode != self.Mode.IDLE:
                    self.main_server.send(b'KO')
                    return
                self.manual_controller = ManualController(rp2040, nano)
                self.mode = self.Mode.MANUAL
                self.main_server.send(f'OK {M_RECEIVER_PORT} {M_TRANSMITTER_PORT}'.encode())

            elif d[1] == b'0': # stop manual mode
                if self.mode != self.Mode.MANUAL:
                    self.main_server.send(b'KO')
                    return
                self.manual_controller.stop()
                self.manual_controller = None
                self.mode = self.Mode.IDLE
                self.main_server.send(b'OK')
            
            elif d[1] == b'S': 
                if self.mode != self.Mode.MANUAL:
                    self.main_server.send(b'KO')
                    return

                # enable/disable sensors
                # add distance sensing
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
                self.camera_transmitter = CameraTransmitter()
                self.main_server.send(f'OK {CAMERA_PORT}'.encode())
            elif d[1] == '0':
                self.camera_transmitter.close()
                self.camera_transmitter = None
                self.main_server.send(b'OK')

        elif d[0] == 'E': # STOP
            self.main_server.send(b'OK')
            self.close_all()
            self.mode = self.Mode.NOT_CONNECTED

        elif d[0] == 'S': # SHUTDOWN
            self.main_server.send(b'OK')
            self.shutdown()
        
    def close_all(self):
        self.nano.send_power(0, 0)
        if self.camera_transmitter != None:
            self.camera_transmitter.close()
        if self.mode == self.Mode.MANUAL:
            self.manual_controller.stop()
            self.manual_controller = None
        self.main_server.close_connection()
        self.rp2040.set_battery(False)
        self.rp2040.set_encoder(False)
        self.rp2040.set_distance(False)
        
    def shutdown(self):
        print(f"Shutting down, battery: {self.rp2040.battery.voltage}V")
        self.close_all()
        os.system("echo password | sudo -S shutdown now")

class MainServer:
    def __init__(self, host, main_port):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.bind((host, main_port))
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
        
    def get_request(self):
        try:
            data = self.connection.recv(32)
            return data
        except BlockingIOError:
            return None
        
    def send(self, data: str):
        self.connection.send(data.encode())
    
    def close_connection(self):
        if self.connection:
            self.connection.close()
        self.connection = None
        self.connection_addr = None


# class ManualReceiver:
#     def __init__(self):
#         self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
#         self.socket.bind((HOST, M_RECEIVER_PORT))
#         self.socket.setblocking(False)
    
#     def get_command(self):
#         data, _ = self.socket.recv(32)
#         d = data.decode().split(' ')
#         speed = int(d[0])
#         direction = d[1]
#         return speed, direction
    
#     def close(self):
#         self.socket.close()
        

# class ManualTransmitter:
#     def __init__(self):
#         self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
#         self.socket.bind((HOST, M_TRANSMITTER_PORT))
#         self.socket.setblocking(False)
#         self.client = None
#         self.last_time = 0

#     def send_data(self, encoder_odometry, obstacle_distance):
#         if self.client == None:
#             _, addr = self.socket.recvfrom(5)
#             self.client = (addr[0], int(addr[1]))
#         message = f'P {encoder_odometry.vx} {encoder_odometry.vt} {encoder_odometry.x} {encoder_odometry.y} {encoder_odometry.t}'
#         message += f' D {obstacle_distance[0]} {obstacle_distance[1]} {obstacle_distance[2]} {obstacle_distance[3]}'
#         self.socket.sendto(message.encode(), self.client)
#         self.last_time = time.time()

#     def close(self):
#         self.socket.close()



if __name__ == "__main__":
    main = Main()
    main.loop()