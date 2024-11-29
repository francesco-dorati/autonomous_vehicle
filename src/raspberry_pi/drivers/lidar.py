"""
    LIDAR class
    handles communication with lidar

    Public Methods:
    - get_health()
        returns status code
    - start_scan()
        starts scan thread
    - stop_scan()
        stops scan thread
    - get_scan()
        returns scan copy

    

"""

import serial
import time
import threading
import struct
from enum import Enum

from raspberry_pi.structures.maps import LocalMap


class Scan:
    def __init__(self):
        """ SCAN: 360 elements array? or something else? """
        self._scan = [0 for _ in range(360)] 

    def add_sample(self, angle_deg: float, dist_mm: int, scan_n: int) -> None:
        """
        Add Sample
        Adds sample to the scan

        Args:
            angle_deg (float): angle of the sample
            dist_mm (int): distance of the obstacle
            scan_n (int): number of the scan
        """
        pass

    def get_local_map(self) -> LocalMap:
        """
        Gat Local Map
        Generates local map based on the scan

        Returns:
            LocalMap: local map based on the scan
        """
        pass

class Lidar:
    PORT = '/dev/ttyUSB0'  
    BAUD_RATE = 460800 
    TIMEOUT = 0.05
    START_COMMAND = b'\xA5\x20'
    STOP_COMMAND = b'\xA5\x25'
    HEALTH_COMMAND = b'\xA5\x52'
    MAX_IN_WAITING = 4095

    def __init__(self):
        self.ser = serial.Serial(self.PORT, self.BAUD_RATE, timeout=self.TIMEOUT, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE)
        self.scanning = False
        self.scan_thread = None

        self._scan = Scan()
        self._sample_index = 0
        self.scan_number = 0
        # self.get_health()

    
    def get_health(self):
        """
        Get Health
        Sends health request and returns status code
            
        Returns:
            int: status code (see lidar docs)
        """
        self.ser.write(self.HEALTH_COMMAND)
        response_descriptor = self.ser.read(7)
        ok, l, m, t = self._unpack_descriptor(response_descriptor)
        if not ok:
            print("ERROR LIDAR (get_health()) Response descriptor not OK")
            return
        # print(f"l: {l}, m: {m}, t: {t}")

        data = self.ser.read(3)
        return int(data[0])

    def start_scan(self):
        """
        Starts scan thread
        """
        # start scanning thread
        self.scan_thread = threading.Thread(target=self._scan_handler)
        self.scan_thread.start()

    def stop_scan(self):
        """
        Stops scan thread and send lidar stop
        """
        print("Stopping LIDAR")
        self.scanning = False
        self.scan_thread.join()
        self.ser.flushInput()
        self.ser.write(self.STOP_COMMAND)
        time.sleep(0.01)
    
    def get_local_map(self) -> LocalMap:
        """
        Get Local Map
        Returns the local map based on the last scan

        Returns:
            LocalMap: local map based on the last scan
        """
        return self._scan.get_local_map() if self.scanning else None

    def _scan_handler(self): # THREAD
        """
        Scan Thread
        Receives lidar data and puts them in _scan
        
        Modifies:
            _scan
        """
        # send start command
        self.ser.flushInput()
        self.ser.write(self.START_COMMAND)

        # receive and check descriptor
        response_descriptor = self.ser.read(7)
        ok, l, _, _, = self._unpack_descriptor(response_descriptor)
        if not ok:
            print("ERROR LIDAR Response descriptor not OK")
            self.scanning = False
            return
        
        # delay for lidar
        time.sleep(0.1)
        self.scanning = True

        # receiver loop
        sample_n = 0
        last_angle = 360.0
        while self.scanning:
            if self.ser.in_waiting < l:
                continue

            # flush to avoid full buffer 
            if self.ser.in_waiting > (510*5 + 5):
                self.ser.read(510*5)
                continue

            # read data
            data = self.ser.read(l)
            if len(data) != l:
                continue

            # unpack data
            s, ns, q, c, angle_deg, dist_mm = self._unpack_data(data)
            
            # validity check
            if not (s ^ ns) or c != 1 or angle_deg > 360.0:
                print("INVALID DATA RECEIVED")
                self.ser.flushInput()
                continue
                # print(f"ERROR {not (s ^ ns)} {c != 1} {angle_deg > 360} {dist_mm == 0.0}")
                # print(f"\n    data: {format(data[0], '08b')} {format(data[1], '08b')} {format(data[2], '08b')} {format(data[3], '08b')} {format(data[4], '08b')}\t waiting: {self.ser.in_waiting}")
                # FIX try to read until it gets a valid data

            # new scan check
            sample_n += 1
            if angle_deg < last_angle:
                # print(f"sample_n: {sample_n}, scan_n: {self.scan_number}, scan_index: {self._sample_index}")
                # print("\n\nNEW SCAN")
                sample_n = 0
                self._sample_index = 0
                self.scan_number += 1
            
            #  save sample to the scan 
            self._scan.add_sample(angle_deg, dist_mm, self.scan_number)
            last_angle = angle_deg

            # print(f"    deg: {angle_deg:.2f}°\t\tdist: {dist_mm} mm\t    waiting: {self.ser.in_waiting}")


        # print("THREAD END")

    
    def _unpack_data(self, data):
        """ 
        Unpacks lidar data

        Args:
            data:   raw data
        Returns:
            s:          start flag
            ns:         not start flag
            q:          quality
            c:          check bit
            angle_deg:  angle in degrees
            dist_mm:    distance in mm
        """
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

    def _unpack_descriptor(self, descriptor):
        """
        Unpacks descriptor

        Args:
            descriptor:    raw data

        Returns:
            ok (bool):     descriptor ok
            data_len,
            data_mode,
            data_type
        """
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
    
    def _add_sample_to_scan(self, sample): # sample_index, sample
        """
        Adds sample to the scan

        Args:
            sample: [angle, dist, number]

        """
        # als cen be pun in a fixed size queue
        # one element per degree
        angle = sample[0]
        dist = sample[1]
        scan_n = sample[2]
        end = lambda: self._sample_index >= len(self._scan)

        # find correct place
        while not end() and self._scan[self._sample_index][0] < angle: 
            # set to 0 if sample is old
            if self._scan[self._sample_index][2] < scan_n - 5:
                self._scan[self._sample_index] = (self._scan[self._sample_index][0], 0, scan_n)
            self._sample_index += 1

        # add sample
        if dist != 0.0:
            if end():
                self._scan.append(sample)
            else:
                self._scan[self._sample_index] = sample
        
        self._sample_index += 1
            






        # self.ser.close()
    

