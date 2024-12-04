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

    LIDAR info:
    response descriptor:
    1 byte          A5
    1 byte          5A
    30 bits         data response length (SCAN_LEN_BYTES???)
    2 bits          send mode
    1 byte          data type
    
    scan data packets: 
    S               (1 bit)
    !S              (1 bit)
    Quality         (6 bit)
    C (checksum)    (1 bit)
    angle           (15 bit)
    distance        (16 bit)
        total: 5 byte


"""

import serial
import time
import threading
import struct
from enum import Enum
import numpy as np
import math

from raspberry_pi.data_structures.maps import LocalMap
from raspberry_pi.utils import timing_decorator

""" FIX SCAN ANGLE -> POSITIVE NEGATIVE ARE DIFFERENT FROM ROBOT'S """
class Scan:
    MAX_SCAN_AGE = 5
    def __init__(self):
        self._last_scan_id = 0
        self._scan = np.full(360, 0, dtype=tuple) # tuple: dist_mm, scan_id
        self._scan_n = np.full(360, 0, dtype=tuple) # tuple: dist_mm, scan_id

    def add_sample(self, angle_deg: float, dist_mm: int, scan_n: int) -> None:
        """
        Add Sample
        Adds sample to the scan

        Args:
            angle_deg (float): angle of the sample, 
            dist_mm (int): distance of the obstacle
            scan_n (int): number of the scan
        """
        self._last_scan_id = scan_n
        angle_deg = math.round(angle_deg)
        self._scan[angle_deg] = (dist_mm, self._last_scan_id)

    def create_local_map(self) -> LocalMap:
        """
        Gat Local Map
        Generates local map based on the scan

        Returns:
            LocalMap: local map based on the scan
        """
        self._clean_scan()
        scan = self._scan[:, 0]
        return LocalMap(scan)


    def _clean_scan(self) -> None:
        for i, s in enumerate(self._scan):
            if s[1] <= self._last_scan_id - self.MAX_SCAN_AGE:
                self._scan[i] = 0


class Lidar:
    PORT = '/dev/ttyUSB0'  
    BAUD_RATE = 460800 
    TIMEOUT = 0.05
    START_COMMAND = b'\xA5\x20'
    STOP_COMMAND = b'\xA5\x25'
    HEALTH_COMMAND = b'\xA5\x52'
    BUFFER_SIZE = 4095
    SCAN_PACKET_SIZE = 5 # bytes
    SCAN_PACKETS_PER_TIME = 600

    _serial = None
    _scanning = False
    _thread = None

    _scan = None
    
    @staticmethod
    @timing_decorator
    def start():
        Lidar._serial = serial.Serial(Lidar.PORT, Lidar.BAUD_RATE, 
                                      timeout=Lidar.TIMEOUT, 
                                      parity=serial.PARITY_NONE, 
                                      stopbits=serial.STOPBITS_ONE)
    @staticmethod   
    @timing_decorator
    def stop():
        if Lidar._scanning:
            Lidar.stop_scan()
        Lidar._thread = None

        Lidar._serial.close()
        Lidar._serial = None

    # def __init__(self):
    #     self.ser = serial.Serial(self.PORT, self.BAUD_RATE, timeout=self.TIMEOUT, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE)
    #     self.scanning = False
    #     self.scan_thread = None

    #     self._scan = Scan()
    #     self._sample_index = 0
    #     self.scan_number = 0
        # self.get_health()

    @staticmethod
    @timing_decorator
    def get_health():
        """
        Get Health
        Sends health request and returns status code
            
        Returns:
            int: status code (see lidar docs)
        """
        if not Lidar._serial:
            print("ERROR LIDAR must run Lidar.start()")
            return
        Lidar._serial.write(Lidar.HEALTH_COMMAND)
        response_descriptor = Lidar._serial.read(7)
        ok, l, m, t = Lidar._unpack_descriptor(response_descriptor)
        if not ok:
            print("ERROR LIDAR (get_health()) Response descriptor not OK")
            return

        data = Lidar._serial.read(3)
        return int(data[0])
    
    @staticmethod
    @timing_decorator
    def start_scan():
        """
        Initializes scan and starts scan thread 
        """
        if not Lidar._serial:
            print("ERROR LIDAR must run Lidar.start()")
            return

        Lidar._scan = Scan()
        Lidar._scanning = True

        # send start command
        Lidar._serial.flushInput()
        Lidar._serial.write(Lidar.START_COMMAND)

        # receive and check descriptor
        response_descriptor = Lidar._serial.read(7)
        ok, l, _, _, = Lidar._unpack_descriptor(response_descriptor)
        if not ok or l != Lidar.SCAN_PACKET_SIZE:
            s1 = "ERROR LIDAR Response descriptor not OK"
            s2 = "ERROR LIDAR Length mismatch"
            print(s1 if not ok else s2)
            Lidar._scanning = False
            return
        time.sleep(0.1)

        # start scanning thread
        Lidar._thread = threading.Thread(target=Lidar._scan_handler)
        Lidar._thread.start()

    @staticmethod
    @timing_decorator
    def stop_scan():
        """
        Stops scan thread and send lidar stop
        """
        if Lidar._scanning:
            Lidar._scanning = False
            Lidar._thread.join()
        Lidar._scan = None
        
        Lidar._serial.flushInput()
        Lidar._serial.write(Lidar.STOP_COMMAND)
        time.sleep(0.01)
    
    @staticmethod
    @timing_decorator
    def is_scanning():
        return Lidar._scanning
    
    @staticmethod
    @timing_decorator
    def create_local_map() -> LocalMap:
        """
        Create Local Map
        Returns the local map based on the last scan

        Returns:
            LocalMap: local map based on the last scan
        """
        return Lidar._scan.create_local_map() if Lidar._scanning else None

    @staticmethod
    def _scan_handler(): # THREAD
        """
        Scan Thread
        Receives lidar data and puts them in _scan
        
        Modifies:
            _scan
        """
        scan_n = 0
        with open("output.txt", "w") as f:
            while Lidar._scanning:
                # if Lidar._serial.in_waiting == 4095:
                #     break
                # print(Lidar._serial.in_waiting)
                bytes_to_read = Lidar.SCAN_PACKET_SIZE*Lidar.SCAN_PACKETS_PER_TIME
                if Lidar._serial.in_waiting < bytes_to_read:
                    continue

                f.write("\nin waiting: " + str(Lidar._serial.in_waiting) + "\n")
                f.write("scan number: " + str(scan_n) + "\n")

                # read 5*n bytes
                bytes_receaved = Lidar._serial.read(bytes_to_read)
                if len(bytes_receaved) != bytes_to_read:
                    print("ERROR LIDAR read different bytes than expected")
                    return

                # interpret data -> check if correct order
                samples = Lidar._process_scan_bytes(bytes_receaved)
                if not samples:
                    print(samples)
                    print("ERROR LIDAR process_bytes error")
                    return
                
                for s in samples:
                    f.write(f"{int(s[0])} {int(s[1])}\n")
                scan_n += 1

                Lidar._scan.add_sample()

            # check if the head is correct
            # divide in packets
            # read each packet
            
            # add to scan 

        
        
        return



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

    @staticmethod
    def _process_scan_bytes(bytes_received):
        # check if order correct
        i = 0
        packets_number = math.floor(len(bytes_received)/Lidar.SCAN_PACKET_SIZE)
        scan = []
        while i < packets_number:
            from_byte = i*Lidar.SCAN_PACKET_SIZE
            to_byte = (i+1)*Lidar.SCAN_PACKET_SIZE
            s, ns, q, c, angle, dist = Lidar._unpack_packet(bytes_received[from_byte:to_byte])
            if s == ns or c == 0:
                print(f"ERROR {i}")
                break
            scan.append((angle, dist))
            i += 1
        return scan
    


        
                # WRONG ORDER
            

        
        # 
        
    @staticmethod
    def _unpack_packet(data):
        """ 
        Unpacks lidar data

        Args:
            data:   raw data
        Returns:
            s:          start flag
            ns:         not start flag
            q:          quality
            c:          check bit
            angle_deg:  angle in degrees (left positive)
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
        angle_deg = 360 - (angle_q6 / 64.0)  # Convert to degrees and 

        distance_low = data[3]  # distance_q2[7:0] 
        distance_high = data[4]  # distance_q2[15:8]
        distance_q2 = (distance_high << 8) | distance_low  # Combine high and low bytes for distance
        dist_mm = distance_q2 / 4.0  # Convert to mm

        return s, ns, q, c, angle_deg, dist_mm

    @staticmethod
    def _unpack_descriptor(descriptor):
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
    
    # def _add_sample_to_scan(self, sample): # sample_index, sample
    #     """
    #     Adds sample to the scan

    #     Args:
    #         sample: [angle, dist, number]

    #     """
    #     # als cen be pun in a fixed size queue
    #     # one element per degree
    #     angle = sample[0]
    #     dist = sample[1]
    #     scan_n = sample[2]
    #     end = lambda: self._sample_index >= len(self._scan)

    #     # find correct place
    #     while not end() and self._scan[self._sample_index][0] < angle: 
    #         # set to 0 if sample is old
    #         if self._scan[self._sample_index][2] < scan_n - 5:
    #             self._scan[self._sample_index] = (self._scan[self._sample_index][0], 0, scan_n)
    #         self._sample_index += 1

    #     # add sample
    #     if dist != 0.0:
    #         if end():
    #             self._scan.append(sample)
    #         else:
    #             self._scan[self._sample_index] = sample
        
    #     self._sample_index += 1
            






        # self.ser.close()
    

