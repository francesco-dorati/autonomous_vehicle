"""
    LIDAR class 
    eredits from Device
    handles communication with lidar
    [static] [thread]
    
    ### PUBLIC ###
    methods:
        start() -> None
            [delay] 
            starts serial connection with lidar
            raises InvalidDescriptor: if health response descriptor not OK
            raises HealthWarning: if health status is WARNING
            raises HealthError: if health status is ERROR
            
        stop() -> None
            [delay] 
            stops serial connection with lidar
            raises ConnectionNotInitiated: if lidar connection not initiated

        ping() -> bool
            [pure] [delay] 
            checks connection with lidar
            raises InvalidDescriptor: if response descriptor not OK
            raises HealthWarning: if health status is WARNING
            raises HealthError: if health status is ERROR
            returns bool: ping successful and health status OK

        start_scan() -> None
            [delay]
            initializes and starts scan thread
            raises ConnectionNotInitiated: if lidar connection not initiated
            raises ConnectionFailed: if ping failed
            raises InvalidDescriptor: if descriptor length mismatch or invalid descriptor

        stop_scan() -> None
            [delay] 
            stops lidar scan

        is_scanning() -> bool
            [pure]
            check if lidar is scanning
            returns bool: lidar is scanning

        produce_local_map() -> LocalMap
            [pure]
            raises NotScanning: if lidar is not scanning
            returns: local map based on the last scan
    
    ### PRIVATE ###
    attributes:
        _serial:    Optional[serial.Serial]
        _thread:    Optional[threading.Thread]
        _scan:      Optional[LidarScan]
        _scanning:  bool

    methods:
        _scan_handler() -> None
            [thread]
            scan thread, receives lidar data and puts them in _scan

        _unpack_descriptor(descriptor: bytes) -> tuple[int, int, int]
            [pure]
            unpacks descriptor
            argument descriptor (bytes): descriptor bytes
            raises InvalidDescriptor: if response descriptor not OK
            returns tuple[int, int, int]: data_len, data_mode, data_type

        _unpack_scan_bytes(bytes_received: bytes) -> list
            [pure]
            upacks scan data bytes, checks data validity and read data
            argument bytes_received (bytes): bytes received from serial
            raises InvalidDataReceived: if data is misaligned
            returns list[tuple[float, float]]: list of (angle, dist)

        _unpack_packet(data: bytes) -> tuple[bool, bool, int, int, float, float]
            [pure]
            upacks single packet
            argument data (bytes): raw data
            returns tuple[bool, bool, int, int, float, float]: 
                s (bool):           start flag
                ns (bool):          not start flag
                q (int):            quality
                c (int):            check bit
                angle_deg (float):  angle in degrees (left positive)
                dist_mm (float):    distance in mm

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
import math
from typing import Optional, List
import numpy as np

from raspberry_pi.data_structures.states import PolarPoint
from raspberry_pi.devices.device import Device
from raspberry_pi.data_structures.maps import LocalMap, LidarScan

from raspberry_pi.config import LIDAR_CONFIG

from raspberry_pi.utils.logger import get_logger, timing_decorator

logger = get_logger(__name__)

class Lidar(Device):
    # PORT = '/dev/ttyUSB0'  
    # BAUD_RATE = 460800 
    # TIMEOUT = 0.05
    # START_COMMAND = b'\xA5\x20'
    # STOP_COMMAND = b'\xA5\x25'
    # HEALTH_COMMAND = b'\xA5\x52'
    # BUFFER_SIZE = 4095
    # SCAN_PACKET_SIZE = 5 # bytes
    # SCAN_PACKETS_PER_TIME = 200

    _serial: Optional[serial.Serial] = None
    _thread: Optional[threading.Thread] = None
    _scan: Optional[LidarScan] = None
    _scanning: bool = False

    class NotScanning(Exception):
        pass
    class InvalidDescriptor(Exception):
        pass
    class InvalidDataReceived(Exception):
        pass
    class HealthWarning(Exception):
        pass
    class HealthError(Exception):
        pass

    @staticmethod
    @timing_decorator
    def start() -> None:
        """
        Starts serial connection with lidar
        Raises:
            InvalidDescriptor: if health response descriptor not OK
            HealthWarning: if health status is WARNING
            HealthError: if health status is ERROR
        """
        # start serial connection
        Lidar._serial = serial.Serial(LIDAR_CONFIG.PORT, LIDAR_CONFIG.BAUD_RATE, 
                                      timeout=LIDAR_CONFIG.TIMEOUT, 
                                      parity=serial.PARITY_NONE, 
                                      stopbits=serial.STOPBITS_ONE)
        time.sleep(0.2)
        # check connection and status
        if not Lidar.ping():
            Lidar.stop()
            logger.error("Lidar ping failed")
            raise Lidar.ConnectionFailed("Health check failed")
        
        logger.info("Lidar connection established")


    @staticmethod   
    @timing_decorator
    def stop() -> None:
        """
        Stops serial connection with lidar
        Raises:
            ConnectionNotInitiated: if lidar connection not initiated
        """
        if not Lidar._serial:
            raise Lidar.ConnectionNotInitiated("Lidar connection not initiated")
        
        
        Lidar.stop_scan()
        Lidar._thread = None

        Lidar._serial.close()
        Lidar._serial = None
        logger.info("Lidar connection closed")


    @staticmethod
    @timing_decorator
    def ping() -> bool:
        """ PURE
        Checks connection with lidar
        Raises:
            InvalidDescriptor: if response descriptor not OK
            HealthWarning: if health status is WARNING
            HealthError: if health status is ERROR
        Returns:
            bool: ping successful and health status OK
        """
        # connection check
        if not Lidar._serial:
            return False
        
        # send health command 
        try:
            Lidar._serial.write(LIDAR_CONFIG.HEALTH_COMMAND)   
        except:
            return False
        
        # read response descriptor
        response_descriptor = Lidar._serial.read(7)
        Lidar._unpack_descriptor(response_descriptor)

        # read status code
        data = Lidar._serial.read(3)
        code = int(data[0])
        if code == 2:
            raise Lidar.HealthError("Lidar health error")
        if code == 1:
            raise Lidar.HealthWarning("Lidar health warning")
             
        return True
    
    @staticmethod
    @timing_decorator
    def start_scan():
        """
        Initializes and starts scan thread
        Raises:
            ConnectionNotInitiated: if lidar connection not initiated
            ConnectionFailed: if ping failed
            InvalidDescriptor: if descriptor length mismatch or invalid descriptor
        """
        # connection check
        if not Lidar._serial:
            raise Lidar.ConnectionNotInitiated("Lidar connection not initiated")
        if not Lidar.ping():
            raise Lidar.ConnectionFailed("Ping failed")
        
        # initialize variables
        Lidar._scan = LidarScan()
        Lidar._scanning = True

        # send start command
        Lidar._serial.flush()
        Lidar._serial.write(LIDAR_CONFIG.START_COMMAND)

        # receive and check descriptor
        response_descriptor = Lidar._serial.read(7)
        l, _, _, = Lidar._unpack_descriptor(response_descriptor)
        if l != LIDAR_CONFIG.SCAN_PACKET_SIZE:
            Lidar.stop()
            logger.error("Lidar descriptor length mismatch")
            raise Lidar.InvalidDescriptor("Descriptor length mismatch")
        
        # delay for lidar to start scanning
        time.sleep(2)

        # start scanning thread
        Lidar._thread = threading.Thread(target=Lidar._scan_handler)
        Lidar._thread.start()

        time.sleep(1)

    @staticmethod
    @timing_decorator
    def stop_scan():
        """
        Stops scan thread and send lidar stop
        """
        # connection check
        if not Lidar._serial:
            raise Lidar.ConnectionNotInitiated("Lidar connection not initiated")
        # if not Lidar.ping():
        #     raise Lidar.ConnectionFailed("Ping failed")
        
        # stop reading
        if Lidar._scanning:
            Lidar._scanning = False
            Lidar._thread.join()
        Lidar._scan = None

        # write stop command
        Lidar._serial.write(LIDAR_CONFIG.STOP_COMMAND)
        time.sleep(0.01)
        Lidar._serial.flush()
    
    @staticmethod
    @timing_decorator
    def is_scanning() -> bool:
        """ PURE
        Check if lidar is scanning
        Returns:
            bool: lidar is scanning
        """
        return Lidar._scanning
    
    @staticmethod
    @timing_decorator
    def produce_local_map() -> LocalMap:
        """ PURE
        Produces LocalMap from current scan
        Raises:
            NotScanning: if lidar is not scanning
        Returns 
            LocalMap: local map based on latest scan
        """
        if not Lidar._scanning:
            raise Lidar.NotScanning("Lidar is not scanning")
        # create local map
        copy: np.array[PolarPoint] = Lidar._scan.get_copy()
        return LocalMap(copy)

    # PRIVATE

    @staticmethod
    def _scan_handler() -> None: # THREAD
        """ TREAD
        Scan thread
        Receives lidar data and puts them in _scan
        """
        try:
            while Lidar._scanning:
                # calculate bytes to read
                bytes_to_read: int = LIDAR_CONFIG.SCAN_PACKET_SIZE * LIDAR_CONFIG.SCAN_PACKETS_PER_TIME
                if Lidar._serial.in_waiting < bytes_to_read:
                    # t = time.time()
                    # print("SCANNING WAIT", Lidar._serial.in_waiting, " < ", bytes_to_read)
                    # print((time.time()-t)*1000)
                    Lidar._serial.reset_input_buffer()
                    time.sleep(0.05)
                    continue
                    
                # read bytes
                bytes_receaved = Lidar._serial.read(bytes_to_read)
                if len(bytes_receaved) != bytes_to_read:
                    logger.error("Lidar read different bytes than expected")
                    return

                # interpret data -> check if correct order
                samples = Lidar._unpack_scan_bytes(bytes_receaved)
                if not samples:
                    logger.error("ERROR LIDAR process_bytes error, samples: ", samples)
                    return
            
                # print(f"SCANNING OK {len(samples)}")
                # add sample to scan
                for angle, dist in samples:
                    if LIDAR_CONFIG.MIN_DIST_MM < dist < LIDAR_CONFIG.MAX_DIST_MM:
                        Lidar._scan.add_sample(angle, dist)
                
                time.sleep(0.05)

        except Lidar.InvalidDataReceived as e:
            logger.error("ERROR IN SCAN THREAD: ", e)
        except Exception as e:
            logger.error("ERROR: ", e)
        finally:
            Lidar._scanning = False
            Lidar.stop_scan()


    @staticmethod
    def _unpack_descriptor(descriptor: bytes) -> tuple[int, int, int]:
        """ PURE
        Unpacks descriptor
        Raises:
            InvalidDescriptor: if response descriptor not OK
        Args:
            descriptor (bytes): descriptor bytes   
        Returns:
            tuple[int, int, int]: data_len, data_mode, data_type
        """
        flag1 = descriptor[0]
        flag2 = descriptor[1]
        if flag1 != 0xA5 or flag2 != 0x5A:
            raise Lidar.InvalidDescriptor("Wrong response descriptor")
        
        data_len = (descriptor[2])
        data_len |= (descriptor[3]) << 8
        data_len |= (descriptor[4]) << 16
        data_len |= (descriptor[5] & 0b00111111) << 24
        data_len = int(data_len)
        data_mode = int((descriptor[5] & 0b11000000) >> 6)
        data_type = int(descriptor[6])
        return data_len, data_mode, data_type

    @staticmethod
    def _unpack_scan_bytes(bytes_received: bytes) -> list[tuple[float, float]]:
        """ PURE
        Upacks scan data bytes
        Checks data validity and read data
        Args:
            bytes_received (bytes): bytes received from serial
        Raises:
            InvalidDataReceived: if data is invalid
        Returns:
            list[tuple[float, float]]: list of (angle, dist)
        """
        scan = []
        packet_size = LIDAR_CONFIG.SCAN_PACKET_SIZE
        packets_number = len(bytes_received) // packet_size
        packet_n = 0
        misalignment = 0
        while packet_n < packets_number:
            # calculate bytes indexes
            from_byte = misalignment + packet_n * packet_size
            to_byte = misalignment + (packet_n+1) * packet_size
            if to_byte >= len(bytes_received):
                break

            # unpack packet
            s, ns, q, c, angle, dist = Lidar._unpack_packet(bytes_received[from_byte:to_byte])
            
            # misalignment check
            if s == ns or c == 0:
                if misalignment == packet_size:
                    Lidar._serial.reset_input_buffer()
                    break
                    # raise Lidar.InvalidDataReceived("Invalid data received")
                misalignment += 1
                continue 

            # add to scan
            scan.append((angle, dist))
            packet_n += 1

        return scan
        
    @staticmethod
    def _unpack_packet(data) -> tuple[bool, bool, int, int, float, float]:
        """ PURE
        Unpacks single packet
        Args:
            data (bytes):   raw data
        Returns:
            tuple[bool, bool, int, int, float, float]:
                s (bool):           start flag
                ns (bool):          not start flag
                q (int):            quality
                c (int):            check bit
                angle_deg (float):  angle in degrees (left positive)
                dist_mm (float):    distance in mm
        """
        s = bool((data[0] & 0b00000001))  # Start flag (0-1)
        ns = bool((data[0] & 0b00000010) >> 1)  # Inverted start flag (1-2)
        q = int((data[0] >> 2) & 0b00111111)  # Quality (2-8)
        
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


