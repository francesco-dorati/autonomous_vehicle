import time
import socket
import os
from enum import Enum


from rp2040 import RP2040_SER
from nano import NANO
from manual import ManualController
from camera import CameraTransmitter

class Main:
    # CONST
    # host
    HOST = '172.20.10.3'
    MAIN_PORT = 5500
    MANUAL_PORT = 5501
    CAMERA_PORT = 5503

    # delays
    BATTERY_CHECK_INTERVAL = 5 # s
    MAIN_SERVER_INTERVAL = 0.4 # s
    MANUAL_TRANSMITTER_INTERVAL = 0.2 # s
    MANUAL_LOOP_INTERVAL = 0.02 # s
    CAMERA_TRANSMITTER_INTERVAL = 0.04 # s


    class Mode(Enum):
        NOT_CONNECTED = -1
        IDLE = 0
        MANUAL = 1
    
    def __init__(self):
        self.mode = self.Mode.NOT_CONNECTED

        self.main_server = MainServer(self.HOST, self.MAIN_PORT) # handles main connection to the user
        
        self.rp2040 = RP2040_SER() # handles communication with the rp2040 (SENSOR DATA)
        self.nano = NANO() # handles communication with the nano (ACTION EXECUTION)

        self.last_battery_check = 0
        self.last_main_server_check = 0
        self.last_camera_transmitted = 0
        
        self.manual_controller = None
        self.camera_transmitter = None
        
        self.distance_sensing = False
        self.rp2040.set_battery(True)
        # self.rp2040.set_encoders(False)
        # self.rp2040.set_encoders(False)

        print("Main Server started.")

    def loop(self):
        print("Start Loop")

        # start battery sensing

        while True:
            t_start = time.time()
            self.rp2040.updated = False

            #1 BATTERY CHECK
            t_battery = time.time()
            if self.rp2040.battery_on and (t_start - self.last_battery_check) >= self.BATTERY_CHECK_INTERVAL: 
                self.last_battery_check = t_start
                self.rp2040.request_data()
                print(f"Battery Check <{'on' if self.rp2040.battery_on else 'off'}>: {self.rp2040.battery.voltage} V ({self.rp2040.battery.level().name})")
                if self.rp2040.battery_on and self.rp2040.battery.is_critical():
                    self.shutdown()
            dt_battery = time.time() - t_battery

            # 2 MAIN CONNECTION HANDLING
            t_main = time.time()
            if (t_start - self.last_main_server_check) >= self.MAIN_SERVER_INTERVAL:
                self.last_main_server_check = t_start
                if self.mode == self.Mode.NOT_CONNECTED: # not connected
                    ok = self.main_server.check_connection()
                    if ok:
                        self.mode = self.Mode.IDLE
                        print("Connection established.")
                    continue
                
                try: # receive request
                    req = self.main_server.get_request()
                    if req != None:
                        self.handle_request(req)

                except ConnectionResetError:
                    self.main_server.close()
                    self.mode = self.Mode.NOT_CONNECTED
                    continue
            dt_main = time.time() - t_main


            # 3 CONTROL
            t_control = time.time()
            if self.mode == self.Mode.MANUAL:
                self.manual_controller.compute()
            dt_control = time.time() - t_control

            # 4 update camera
            t_camera = time.time()
            if self.camera_transmitter != None and (t_start - self.last_camera_transmitted) >= self.CAMERA_TRANSMITTER_INTERVAL:
                self.last_camera_transmitted = t_start
                self.camera_transmitter.send_frame()
            dt_camera = time.time() - t_camera


            # 5 set delay
            print(f"bat: {dt_battery*1000:.1f}\tmain:{dt_main*1000:.1f}\tmanual:{dt_control*1000:.1f}\tcam:{dt_camera*1000:.1f}")
            dt = time.time() - t_start
            dt_max = 0
            if (self.mode == self.Mode.NOT_CONNECTED or self.mode == self.Mode.IDLE):
                if (self.camera_transmitter != None):
                    dt_max = self.CAMERA_TRANSMITTER_INTERVAL
                else:
                    dt_max = self.MAIN_SERVER_INTERVAL
            elif self.mode == self.Mode.MANUAL:
                if self.manual_controller != None:
                    dt_max = self.MANUAL_LOOP_INTERVAL
                else:
                    dt_max = min(self.MANUAL_LOOP_INTERVAL, self.CAMERA_TRANSMITTER_INTERVAL)
            over = dt >= dt_max
            print(f"dt: {(dt*1000):.1f} ms,    ", end='')
            print(f"delay: {(dt_max*1000):.1f} ms {'OVER' if over else ''}")
            time.sleep(dt_max - dt) if not over else None

            


    
        
    def handle_request(self, data):
        if data == b'':
            return

        print("Main Request", data)
        data = data.decode().split('\n')[:-1]
        for d in data:
            d = d.strip().split(' ')
            print("command: ", d)

            if d[0] == 'P': # PING
                self.rp2040.request_data()
                self.main_server.send('P ' + str(self.rp2040.battery.voltage) + ' ' + self.rp2040.battery.level().name)

            elif d[0] == 'M': # MANUAL
                if d[1] == '1': # start manual mode
                    if self.mode != self.Mode.IDLE:
                        self.main_server.send('KO')
                        return
                    self.manual_controller = ManualController(self.rp2040, self.nano, self.HOST, self.MANUAL_PORT)
                    self.mode = self.Mode.MANUAL
                    self.main_server.send(f'OK {self.MANUAL_PORT}')
                    print("MANUAL START")

                elif d[1] == '0': # stop manual mode
                    if self.mode != self.Mode.MANUAL:
                        self.main_server.send('KO')
                        return
                    self.manual_controller.stop()
                    self.manual_controller = None
                    self.mode = self.Mode.IDLE
                    self.main_server.send('OK')
                    print("MANUAL STOP")
                
                # elif d[1] == 'S': 
                #     if self.mode != self.Mode.MANUAL:
                #         self.main_server.send('KO')
                #         return

                #     # enable/disable sensors
                #     # add distance sensing
                #     for e in d[2:]:
                #         if e[0] == 'E':
                #             # encoders
                #             set_encoder_odometry(int(e[1]))
                #             pass
                #         elif e[0] == 'D':
                #             # distance
                #             set_distance_sensing(int(e[1]))
                #             pass
                #         else:
                #             main_server.send(b'KO')
                #             return
                        
            elif d[0] == 'C': # CAMERA
                if d[1] == '1':
                    self.camera_transmitter = CameraTransmitter(self.HOST, self.CAMERA_PORT)
                    self.main_server.send(f'OK {self.CAMERA_PORT}')
                elif d[1] == '0':
                    self.camera_transmitter.close()
                    self.camera_transmitter = None
                    self.main_server.send('OK')

            elif d[0] == 'E': # STOP
                self.close_all()
                self.mode = self.Mode.NOT_CONNECTED

            elif d[0] == 'S': # SHUTDOWN
                self.shutdown()
        
    def close_all(self):
        self.nano.send_power(0, 0)
        if self.camera_transmitter != None:
            self.camera_transmitter.close()
        if self.mode == self.Mode.MANUAL:
            self.manual_controller.stop()
            self.manual_controller = None
        self.main_server.close_connection()
        self.rp2040.set_encoder(False)
        self.rp2040.set_distance(False)
        
    def shutdown(self):
        print(f"Shutting down.\n\n")
        self.close_all()
        os.system("echo password | sudo -S shutdown now")

class MainServer:
    def __init__(self, host, main_port):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket.bind((host, main_port))
        self.socket.setblocking(False)
        self.socket.listen(1)

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