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
        ok = self.unpack_descriptor(response_descriptor)
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
        ok = self.unpack_descriptor(response_descriptor)
        if not ok:
            print("Response descriptor not OK")
            self.scanning = False
            return
        
        print("Scanning Thread:")
        while self.scanning:
            data = self.ser.read(5)
            if len(data) != 5:
                print(f"LIDAR ignored data: {data}")
                continue

            self.unpack_data(data)
    
    def unpack_data(self, data):
        s = bool((data[0] & 0b00000001))  # Start flag (0-1)
        ns = bool((data[0] & 0b00000010) >> 1)  # Inverted start flag (1-2)
        quality = (data[0] >> 2) & 0b00111111  # Quality (2-8)
        print(f"data 0: {format(data[0], '08b') }, s: {s}, ns: {ns},  q: {quality}")
        print(f"data 1: {format(data[1], '08b') }")
        print(f"data 2: {format(data[2], '08b') }")
        print(f"data 3: {format(data[3], '08b') }")
        
        c = (data[0] & 0b01000000) >> 6  # Check bit (8-9)
        angle_q6_low = data[1]  # angle_q6[6:0] (9-16)
        angle_q6_high = data[2]  # angle_q6[14:7] (16-24)
        distance_low = data[3]  # distance_q2[7:0] (24-32)
        distance_high = data[4]  # distance_q2[15:8] (32-40)

        # Calculate actual angle and distance
        angle_q6 = (angle_q6_high << 7) | angle_q6_low  # Combine high and low bytes for angle
        angle = angle_q6 / 64.0  # Convert to degrees
        distance_q2 = (distance_high << 8) | distance_low  # Combine high and low bytes for distance
        distance = distance_q2 / 4.0  # Convert to mm

        return s

        
    def unpack_descriptor(self, descriptor):
        flag1 = descriptor[0]
        flag2 = descriptor[1]
        if flag1 != b'\xA5' or flag2 != b'\x5A':
            print(f"Wrong response descriptor: {descriptor}")
            return False
        
        data_len_mode = (descriptor[2] << 24) | (descriptor[3] << 16) | (descriptor[4] << 8) | descriptor[5]
        data_len = (data_len_mode >> 2) & 0x3FFFFFFF 
        mode = data_len_mode & 0x3  
        data_type = descriptor[6]
        print(f"Response descriptor OK: \n\tdata len: {data_len}, mode: {mode}, type: {data_type}")
        return True
    

    def stop(self):
        print("sending STOP")
        self.ser.write(self.STOP_COMMAND)
        self.scanning = False
    

