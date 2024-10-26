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
    STOP_COMMAND = b'\xA5\x25'
    HEALTH_COMMAND = b'\xA5\x52'

    def __init__(self):
        self.ser = serial.Serial(self.PORT, self.BAUD_RATE, timeout=self.TIMEOUT, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE)
        self.scanning = False
        self.scan_thread = None
        self.scan = []
        # self.get_health()

    
    def get_health(self):
        self.ser.write(self.HEALTH_COMMAND)
        response_descriptor = self.ser.read(7)
        ok, l, m, t = self.unpack_descriptor(response_descriptor)
        if not ok:
            print("Health response descriptor not OK")
            return
        print(f"l: {l}, m: {m}, t: {t}")

        data = self.ser.read(3)
        status = data[0]
        print(f"Status: {status}")

    def start_scan(self):
        # start scanning thread
        self.scan_thread = threading.Thread(target=self.scan_data_handler)
        self.scan_thread.start()

    def scan_data_handler(self): # THREAD
        print("THREAD START")
        # send start command
        self.ser.flushInput()
        self.ser.write(self.START_COMMAND)
        response_descriptor = self.ser.read(7)
        ok, l, _, _, = self.unpack_descriptor(response_descriptor)
        if not ok:
            print("Response descriptor not OK")
            self.scanning = False
            return
        
        time.sleep(0.1)
        self.scanning = True

        sample_n = 0
        scan_index = 0
        last_angle = 360.0
        while self.scanning:
            if self.ser.in_waiting < l:
                continue

            # read data
            data = self.ser.read(l)
            if len(data) != l:
                continue
            #print(f"\n    data: {format(data[0], '08b')} {format(data[1], '08b')} {format(data[2], '08b')} {format(data[3], '08b')} {format(data[4], '08b')}\t waiting: {self.ser.in_waiting}")

            # unpack data
            s, ns, q, c, angle_deg, dist_mm = self.unpack_data(data)
            
            # validity check
            if not (s ^ ns) or c != 1 or angle_deg > 360.0:
                # print(f"ERROR {not (s ^ ns)} {c != 1} {angle_deg > 360} {dist_mm == 0.0}")
                # FIX try to read until it gets a valid data
                self.ser.flushInput()
                continue

            # new scan check
            i += 1
            if angle_deg < last_angle:
                i = 0
                scan_index = 0
                # print(f"i: {i}")
                # print("\n\nNEW SCAN")
            
            last_angle = angle_deg
            # add to last scan
            if  len(self.scan) <= scan_index:
                self.scan.append((angle_deg, dist_mm))

            elif self.scan[scan_index][0] >= angle_deg:
                # add it to the scan
                self.scan[scan_index] = (angle_deg, dist_mm)
             
            scan_index += 1
         

            # print(f"    deg: {angle_deg:.2f}°\t\tdist: {dist_mm} mm\t    waiting: {self.ser.in_waiting}")


        print("THREAD END")

    
    def unpack_data(self, data):
        s = bool((data[0] & 0b00000001))  # Start flag (0-1)
        ns = bool((data[0] & 0b00000010) >> 1)  # Inverted start flag (1-2)
        q = int((data[0] >> 2) & 0b00111111)  # Quality (2-8)
        # print(f"data 0: {format(data[0], '08b') }, s: {s}, ns: {ns}, ok: {s != ns}, quality: {q}")

        c = int(data[1] & 0b00000001)  # Check bit
        angle_q6_low = (data[1] & 0b11111110) >> 1  # angle_q6[6:0] 
        angle_q6_high = data[2]  # angle_q6[14:7]
        angle_q6 = (angle_q6_high << 7) | angle_q6_low
        angle_deg = angle_q6 / 64.0  # Convert to degrees

        distance_low = data[3]  # distance_q2[7:0] 
        distance_high = data[4]  # distance_q2[15:8]
        distance_q2 = (distance_high << 8) | distance_low  # Combine high and low bytes for distance
        dist_mm = distance_q2 / 4.0  # Convert to mm

        return s, ns, q, c, angle_deg, dist_mm


        
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
    

    def stop_scan(self):
        print("Stopping LIDAR")
        self.scanning = False
        self.ser.write(self.STOP_COMMAND)
        time.sleep(0.01)
        self.ser.flushInput()
    
    def get_scan(self):
        return self.scan.copy()

        # self.ser.close()
    

