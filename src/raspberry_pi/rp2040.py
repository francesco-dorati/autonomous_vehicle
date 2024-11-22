"""
    This module is responsible for handling the RP2040 microcontroller communication.
    It is a part of the RaspberryPi module.


"""

import serial

class RP2040:
    def __init__(self):
        self.serial = serial.Serial('/dev/ttyAMA2', 115200, timeout=0.05)
    
    def stop_motors(self):
        self.serial.send("STP")

    def set_target_power(self, power_left: int, power_right: int):
        """
            Set the motor power.
            power_led: int [-255, 255], 
            power_right: int [-255, 255]
        """
        if power_left == 0 and power_right == 0:
            self.stop_motors()

        self.serial.write(f"POW {power_left} {power_right}\n")
        
    
    def set_target_velocity(self, lin_vel: int, ang_vel: int):
        """
            Set the robot velocity.
            lin_vel: int (mm/s), 
            ang_vel: int (mrad/s)
        """
        if lin_vel == 0 and ang_vel == 0:
            self.stop_motors()

        self.serial.write(f"VEL {lin_vel} {ang_vel}\n")

    def change_pid_values(self, kp: float, ki: float, kd: float):
        """
            TESTING
            Change PID values of velocity controller.
            kp: float, 
            ki: float, 
            kd: float
        """
        self.serial.write(f"PID {kp} {ki} {kd}\n")
        pass

    def request_odometry(self):
        """
            Request odometry data from RP2020
            returns:
                x: int (mm)
                y: int (mm)
                th: int (mrad)
                vx: int (mm/s)
                va: int (mrad/s)
        """
        self.serial.write("ORQ\n")
        line = self.serial.readline().decode().strip().split(" ")
        x = int(line[1])
        y = int(line[2])
        th = int(line[3])
        vx = int(line[4])
        va = int(line[5])
        return x, y, th, vx, va

    def reset_odometry(self):
        """
            Sends reset command
        """
        self.serial.write("ORS\n")

    def set_odometry():
        pass

    def follow_path(self, path):
        pass

    def append_path(self, path):
        pass
