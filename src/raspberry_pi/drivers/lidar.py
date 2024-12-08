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
    30 bits         data response length (SCAN_PACKET_SIZE)
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
import numpy as np
import math

from raspberry_pi.utils import timing_decorator
from raspberry_pi.data_structures.maps import LocalMap
from raspberry_pi.data_structures.lidar_scan import LidarScan

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
    _thread = None
    _scan = None
    _scanning = False

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


    @staticmethod
    @timing_decorator
    def health():
        """
        Get Health
        Sends health request and returns status code
            
        Returns:
            int: status code (see lidar docs)
                    0: OK
                    1: WARNING
                    2: ERROR
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
        
        # initialize variables
        Lidar._scan = LidarScan()
        Lidar._scanning = True

        # send start command
        Lidar._serial.flush()
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
    
        Lidar._serial.flush()
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
        return LocalMap(Lidar._scan) if Lidar._scanning else None

    @staticmethod
    def _scan_handler(): # THREAD
        """
        Scan Thread
        Receives lidar data and puts them in _scan
        
        Modifies:
            _scan
        """
        while Lidar._scanning:
            # calculate bytes to read
            bytes_to_read = Lidar.SCAN_PACKET_SIZE*Lidar.SCAN_PACKETS_PER_TIME
            if Lidar._serial.in_waiting < bytes_to_read:
                continue
                
            # read bytes
            bytes_receaved = Lidar._serial.read(bytes_to_read)
            if len(bytes_receaved) != bytes_to_read:
                print("ERROR LIDAR read different bytes than expected")
                return

            # interpret data -> check if correct order
            samples = Lidar._unpack_scan_bytes(bytes_receaved)
            if not samples:
                print(samples)
                print("ERROR LIDAR process_bytes error")
                return
           
            # add sample to scan
            for angle, dist in samples:
                Lidar._scan.add_sample(angle, dist)

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
        return True, data_len, data_mode, data_type

    @staticmethod
    def _unpack_scan_bytes(bytes_received: bytes) -> list:
        """
        Interpret Data
        Checks data validity and read data
        # TODO extend to handle data disalinement

        Args:
            bytes_received (bytes): bytes received from serial

        Returns:
            list: list of (angle, dist)
        """

        packet = 0
        packets_number = math.floor(len(bytes_received)/Lidar.SCAN_PACKET_SIZE)
        scan = []
        while packet < packets_number:
            from_byte = packet*Lidar.SCAN_PACKET_SIZE
            to_byte = (packet+1)*Lidar.SCAN_PACKET_SIZE
            s, ns, q, c, angle, dist = Lidar._unpack_packet(bytes_received[from_byte:to_byte])
            if s == ns or c == 0:
                print(f"ERROR DATA MISALINED at packet {packet}")
                break
            scan.append((angle, dist))
            packet += 1
        return scan
        
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
        angle_deg = (360 - (angle_q6 / 64.0)) % 360 # Convert to degrees and 

        distance_low = data[3]  # distance_q2[7:0] 
        distance_high = data[4]  # distance_q2[15:8]
        distance_q2 = (distance_high << 8) | distance_low  # Combine high and low bytes for distance
        dist_mm = distance_q2 / 4.0  # Convert to mm

        return s, ns, q, c, angle_deg, dist_mm


