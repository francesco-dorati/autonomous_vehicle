"""
    This module is responsible for handling the RP2040 microcontroller communication.
    It is a part of the RaspberryPi module.


"""

import serial

from raspberry_pi.data_structures.state import Position


class RP2040:
    _serial = serial.Serial('/dev/ttyAMA0', 115200, timeout=0.05)
    
    @staticmethod
    def ping() -> bool:
        RP2040._serial.flush()
        RP2040._serial.write("PNGG\n".encode())
        png = RP2040._serial.read_until("\n")
        print(png)
        png = png.decode()
        print(png)
        png = png.strip()
        print(png)
        return True if png == "PNG" else False

    @staticmethod
    def stop_motors() -> None:
        RP2040._serial.write("STP\n".encode())

    @staticmethod
    def set_target_power(power_left: int, power_right: int) -> None:
        """
        Set the motor power.

        Args:
            power_left (int): [-255, 255], 
            power_right (int): [-255, 255]
        Side:
            writes to serial
        """
        if power_left == 0 and power_right == 0:
            RP2040.stop_motors()

        RP2040._serial.write(f"POW {power_left} {power_right}\n".encode())
        
    @staticmethod
    def set_target_velocity(lin_vel: int, ang_vel: int) -> None:
        """
        Set the robot velocity.

        Args:
            lin_vel (int): mm/s 
            ang_vel (int): mrad/s
        Side:
            writes to serial
        """
        if lin_vel == 0 and ang_vel == 0:
            RP2040.stop_motors()
        RP2040._serial.write(f"VEL {lin_vel} {ang_vel}\n".encode())

    @staticmethod
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
        RP2040._serial.write(f"PID {kp} {ki} {kd}\n".encode())

    @staticmethod
    def get_position() -> Position:
        """
        Request odometry data from RP2020

        Side:
            writes to serial
            awaits response
        Returns:
            Position: position of the robot
        """
        RP2040._serial.write("ORQ\n".encode())
        line = RP2040._serial.readline()
        if not line:
            return None
        
        line = line.decode().strip().split(" ")
        print(line)
        x = int(line[0])
        y = int(line[1])
        th = int(line[2])
        p = Position(x, y, th)
        return p

    @staticmethod
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
        RP2040._serial.write("ODB\n".encode())
        line = RP2040._serial.readline().decode().strip().split(" ")
        x = int(line[0])
        y = int(line[1])
        th = int(line[2])
        vl = int(line[3])
        va = int(line[4])
        return x, y, th, vl, va
        
    @staticmethod
    def reset_position() -> None:
        """
            Sends odometry reset command
        """
        RP2040._serial.write("ORS\n".encode())

    @staticmethod
    def set_position(pos: Position) -> None:
        RP2040._serial.write(f"OST {pos.x} {pos.y} {pos.th}\n".encode())

        pass
    
    @staticmethod
    def follow_path(path):
        pass
    
    @staticmethod
    def append_path(path):
        pass
