import time
import math
import json
import threading
import socket

MANUAL_LOOP_FREQ = 50 # Hz
MANUAL_TAO = (1/MANUAL_LOOP_FREQ) # s

MANUAL_LIN_VEL = [13, 18, 25, 30] # cm/s
MANUAL_ANG_VEL = [30, 60, 90, 120] # deg/s

WHEEL_RADIUS = 3.4
DIST_FROM_CENTER = 12
TICKS_PER_REV = 1495


class ManualController(threading.Thread):
    def __init__(self, hostname, port, serial):
        super().__init__(daemon=True)

        self.hostname = hostname
        self.port = port
        self.serial = serial

        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind((self.hostname, self.port))
        print(f"\n[MANUAL SERVER] Listening on port {self.port}")
        # self.connected = False
        # self.serial.start()


    def run(self):
        while True:
            data, addr = self.socket.recvfrom(1024)
            t_start = time.time()

            print(f"[MANUAL SERVER] Received \"{data.decode()}\"")
            keyboard_buffer = data.decode()
            if keyboard_buffer == "E" or keyboard_buffer == "EXIT":
                self.socket.close()
                break

            lin_vel, ang_vel = self._calculate_speed(keyboard_buffer)

            self.serial.send(lin_vel, ang_vel)
            
            s = self.serial.read()
            data = self.process_data(s, t_start)
            
            self.socket.sendto(json.dumps(data).encode(), addr)


        



        # receive data from socket
        # calculate velocity
        # send to serial

        # skip = 0
        # lin_vel, ang_vel = (0, 0)
        # while self.socket.connected:
        #     t_start = time.time()
        #     received = False

        #     if not self.socket.queue.empty():
        #         skip = 0

        #         keyboard_buffer = self.socket.queue.get()
        #         received = True

        #         lin_vel, ang_vel = self._calculate_speed(keyboard_buffer)
            
        #     self.serial.send(lin_vel, ang_vel)
        #     s = self.serial.read()

        #     if received:
        #         data = self.process_data(s, t_start, lin_vel, ang_vel)
        #         self.socket.send(json.dumps(data))

        #     dt = time.time() - t_start
        #     print(f"[MANUAL] Loop time: {(dt*1000):.3f} ms instead of {(MANUAL_TAO*1000)} ms")
        #     if dt < MANUAL_TAO:
        #         time.sleep(MANUAL_TAO - dt)

    def _calculate_speed(self, keyboard_buffer):
        keyboard_buffer = list(keyboard_buffer)
        speed_level = int(keyboard_buffer[0])
        lin_vel, ang_vel = (0, 0)

        if "f" in keyboard_buffer:
            lin_vel = MANUAL_LIN_VEL[speed_level-1]

        if "b" in keyboard_buffer:
            lin_vel = -MANUAL_LIN_VEL[speed_level-1]

        if "l" in keyboard_buffer:
            ang_vel = MANUAL_ANG_VEL[speed_level-1]

        if "r" in keyboard_buffer:
            ang_vel = -MANUAL_ANG_VEL[speed_level-1]

        return lin_vel, ang_vel

    def process_data(self, data_string, t_start):
        pos, vel, wvel, t, _ = data_string.split(";")
        pos = pos.split(" ")
        vel = vel.split(" ")
        wvel = wvel.split(" ")
        t = t.split(" ")

        # pos, vel, wvel, time ard, time rpi

        data = {}
        data["position"] = [float(pos[1]), float(pos[2]), float(pos[3])]
        data["actual_velocity"] = [float(vel[1]), float(vel[2])]
        data["wheels_velocity"] = [float(wvel[1]), float(wvel[1])]
        data["time_arduino"] = float(t[1])
        data["time_rpi"] = (time.time() - t_start)*1000

        return data
    
        





