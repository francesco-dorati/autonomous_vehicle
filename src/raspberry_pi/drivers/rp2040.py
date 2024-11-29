"""
    This module is responsible for handling the RP2040 microcontroller communication.
    It is a part of the RaspberryPi module.


"""

import serial

from raspberry_pi.structures.state import Position


class RP2040:
    _serial = serial.Serial('/dev/ttyAMA2', 115200, timeout=0.05)
    
    @staticmethod
    def stop_motors() -> None:
        RP2040._serial.send("STP")

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

        RP2040._serial.write(f"POW {power_left} {power_right}\n")
        
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
        RP2040._serial.write(f"VEL {lin_vel} {ang_vel}\n")

    @staticmethod
    def change_pid_values(kp: float, ki: float, kd: float) -> None:
        """
        Change PID values of velocity controller.

        Args:
            kp (float) 
            ki (float) 
            kd (float)
        Side:
            writes to serial
        """
        RP2040._serial.write(f"PID {kp} {ki} {kd}\n")

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
        RP2040._serial.write("ORQ\n")
        line = RP2040.serial.readline().decode().strip().split(" ")
        x = int(line[1])
        y = int(line[2])
        th = int(line[3])
        # vx = int(line[4])
        # va = int(line[5])
        return x, y, th

    @staticmethod
    def reset_position() -> None:
        """
            Sends odometry reset command
        """
        RP2040._serial.write("ORS\n")

    @staticmethod
    def set_position(pos: Position) -> None:
        pass
    
    @staticmethod
    def follow_path(path):
        pass
    
    @staticmethod
    def append_path(path):
        pass
