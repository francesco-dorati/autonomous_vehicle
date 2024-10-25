# Import necessary libraries
import serial
import time
import threading
import struct
from enum import Enum


class Lidar:
    PORT = '/dev/ttyUSB0'  
    BAUD_RATE = 460800 
    TIMEOUT = 0.05
    START_COMMAND = b'\xA5\x20'
    # START_RESPONSE_DESCRIPTOR = b"\xA5\x5A\x05\x00\x00\x40\x81"
    STOP_COMMAND = b'\xA5\x25'
    HEALTH_COMMAND = b'\xA5\x52'
    # HEALTH_RESPONSE_DESCRIPTOR = b"\xA5\x5A\x03\x00\x00\x00\x06"
    # \xa5Z\x03\x00\x00\x00\x06


    def __init__(self):
        self.ser = serial.Serial(self.PORT, self.BAUD_RATE, timeout=self.TIMEOUT)
        self.scanning = False
        # self.get_health()

    
    def get_health(self):
        self.ser.write(self.HEALTH_COMMAND)
        response_descriptor = self.ser.read(7)
        ok, _, _, _ = self.unpack_descriptor(response_descriptor)
        if not ok:
            print("Health response descriptor not OK")
            return

        data = self.ser.read(3)
        status = data[0]
        print(f"Status: {status}")

    def start(self):
        # start scanning thread
        self.scan_thread = threading.Thread(target=self.scan)
        self.scan_thread.start()

    def scan(self): # THREAD
        # send start command
        self.ser.write(self.START_COMMAND)
        response_descriptor = self.ser.read(7)
        ok, _, _, _, = self.unpack_descriptor(response_descriptor)
        if not ok:
            print("Response descriptor not OK")
            self.scanning = False
            return
        
        print("Scanning Thread:")
        self.scanning = True
        while self.scanning:
            data = self.ser.read(5)
            if len(data) != 5:
                print(f"LIDAR ignored data: {data}")
                continue

            self.unpack_data(data)
    
    def unpack_data(self, data):
        s = bool((data[0] & 0b00000001))  # Start flag (0-1)
        ns = bool((data[0] & 0b00000010) >> 1)  # Inverted start flag (1-2)
        q = int((data[0] >> 2) & 0b00111111)  # Quality (2-8)
        # print(f"data 0: {format(data[0], '08b') }, s: {s}, ns: {ns}, ok: {s != ns}, quality: {q}")

        c = (data[1] & 0b00000001)  # Check bit
        angle_q6_low = (data[1] & 0b11111110) >> 1  # angle_q6[6:0] 
        angle_q6_high = data[2]  # angle_q6[14:7]
        angle_q6 = (angle_q6_high << 7) | angle_q6_low
        angle_deg = angle_q6 / 64.0  # Convert to degrees

        distance_low = data[3]  # distance_q2[7:0] 
        distance_high = data[4]  # distance_q2[15:8]
        distance_q2 = (distance_high << 8) | distance_low  # Combine high and low bytes for distance
        dist_mm = distance_q2 / 4.0  # Convert to mm

        # print(f"data 1: {format(data[1], '08b') }")
        # print(f"data 2: {format(data[2], '08b') }")
        # print(f"data 3: {format(data[3], '08b') }")
        print(f"Scan: {'NEW' if s else ''} deg: {angle_deg}°, dist: {dist_mm} mm")
        


        # Calculate actual angle and distance

        return s

        
    def unpack_descriptor(self, descriptor):
        flag1 = descriptor[0]
        flag2 = descriptor[1]
        if flag1 != 0xA5 or flag2 != 0x5A:
            print(f"Wrong response descriptor: {hex(flag1)}, {hex(flag2)}")
            return False, None, None, None
        
        data_len = (descriptor[2])
        data_len |= (descriptor[3]) << 8
        data_len |= (descriptor[4]) << 16
        data_len |= (descriptor[5] & 0b00111111) << 24
        data_len = int(data_len)
        data_mode = int((descriptor[5] & 0b11000000) >> 6)
        data_type = int(descriptor[6])
        # print(f"Response descriptor OK: \n\tdata len: {data_len}, mode: {data_mode}, type: {data_type}")
        return True, data_len, data_mode, data_type
    

    def stop(self):
        print("sending STOP")
        self.ser.write(self.STOP_COMMAND)
        self.scanning = False
    

