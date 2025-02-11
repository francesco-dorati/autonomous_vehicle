"""
    This module is responsible for handling the RP2040 microcontroller communication.
    It is a part of the RaspberryPi module.


"""

import serial
import time
from typing import List
import threading

from raspberry_pi.devices.device import Device
from raspberry_pi.data_structures.states import Position
from raspberry_pi.utils.logger import get_logger, timing_decorator
from raspberry_pi.config import RP2040_CONFIG

logger = get_logger(__name__)

class RP2040(Device):
    _serial_lock = threading.Lock()

    @staticmethod
    @timing_decorator
    def start() -> None:
        try:
            RP2040._serial = serial.Serial(RP2040_CONFIG.PORT, RP2040_CONFIG.BAUD_RATE, timeout=RP2040_CONFIG.TIMEOUT)
            time.sleep(2)
            if not RP2040.ping(): # check connection
                RP2040.stop()
                raise Device.ConnectionFailed
            logger.info("RP2040 connection established.")
        except serial.SerialException:
            raise Device.ConnectionFailed

    @staticmethod
    @timing_decorator
    def stop() -> None:
        if RP2040._serial:
            with RP2040._serial_lock:
                RP2040._serial.close()
        RP2040._serial = None

    @staticmethod
    @timing_decorator
    def ping() -> bool:
        with RP2040._serial_lock:
            try:
                RP2040._serial.write("PNG\n".encode())
                png = RP2040._serial.readline().decode().strip()
                return png == "PNG"
            except Exception as e:
                logger.error(f"Ping failed: {e}")
                return False

    @staticmethod
    @timing_decorator
    def stop_motors() -> None:
        with RP2040._serial_lock:
            RP2040._serial.write("STP\n".encode())

    @staticmethod
    @timing_decorator
    def set_target_power(power_left: int, power_right: int) -> None:
        """
        Set the motor power.

        Args:
            power_left (int): [-255, 255], 
            power_right (int): [-255, 255]
        Side:
            writes to serial
        """
        with RP2040._serial_lock:
            RP2040._serial.write(f"PWR {power_left} {power_right}\n".encode())
        
    @staticmethod
    @timing_decorator
    def set_target_velocity(lin_vel: int, ang_vel: int) -> None:
        """
        Set the robot velocity.

        Args:
            lin_vel (int): mm/s 
            ang_vel (int): mrad/s
        Side:
            writes to serial
        """
        with RP2040._serial_lock:
            RP2040._serial.write(f"VEL {lin_vel} {ang_vel}\n".encode())

    @staticmethod
    @timing_decorator
    def set_pid_values(kp: float, ki: float, kd: float) -> None:
        """
        Change PID values of velocity controller.

        Args:
            kp (float) 
            ki (float) 
            kd (float)
        Side:
            writes to serial
        """
        with RP2040._serial_lock:
            RP2040._serial.write(f"PID {kp} {ki} {kd}\n".encode())

    @staticmethod
    @timing_decorator
    def get_position() -> Position:
        """
        Request odometry data from RP2040

        Side:
            writes to serial
            awaits response
        Returns:
            Position: position of the robot
        """
        with RP2040._serial_lock:
            RP2040._serial.write("ORQ\n".encode())
            line = RP2040._serial.readline().decode().strip()
            if line:
                try:
                    x, y, th = map(int, line.split())
                    return Position(x, y, th)
                except ValueError:
                    logger.error(f"Invalid position data received: {line}")
                    return None
            else:
                logger.error("No data received from RP2040")
                return None

    @staticmethod
    # @timing_decorator
    def get_debug_odometry() -> tuple[int, int, int, int, int]:
        """
        Request debug odometry data from RP2020

        Side:
            writes to serial
            awaits response
        Returns:
            int: x position of the robot    [mm]
            int: y position of the robot    [mm]
            int: theta position of the robot    [??]
            int: vl position of the robot   [mm/s]
            int: va position of the robot   [mm/s]
        """
        with RP2040._serial_lock:
            RP2040._serial.write("ODB\n".encode())
            line = RP2040._serial.readline().decode().strip().split(" ")
            print(line)
            x = int(line[0])
            y = int(line[1])
            th = int(line[2])
            vl = int(line[3])
            va = int(line[4])
            return x, y, th, vl, va
        
    @staticmethod
    @timing_decorator
    def reset_position() -> None:
        """
            Sends odometry reset command
        """
        with RP2040._serial_lock:
            RP2040._serial.write("ORS\n".encode())

    @staticmethod
    @timing_decorator
    def set_position(pos: Position) -> None:
        """Overrides the current position

        Args:
            pos (Position): new position
        """
        with RP2040._serial_lock:
            RP2040._serial.write(f"OST {pos.x} {pos.y} {pos.th}\n".encode())
       
    
    @staticmethod
    @timing_decorator
    def follow_path(path: List[Position]):
        """Sends path to follow

        Args:
            path (List[Position]): path to follow
        """
        with RP2040._serial_lock:
            n = len(path)
            req = f"PTH {n} "
            for pos in path:
                req += f"{pos.x} {pos.y} {pos.th}"
                if pos != path[-1]:
                    req += " "
            print(req)
            RP2040._serial.write(req.encode())
    
    @staticmethod
    @timing_decorator
    def append_path(path: List[Position]):
        with RP2040._serial_lock:
            n = len(path)
            req = f"APP {n} "
            for pos in path:
                req += f"{pos.x} {pos.y} {pos.th}"
                if pos != path[-1]:
                    req += " "
            RP2040._serial.write(req.encode())
