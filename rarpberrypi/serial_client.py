import time
import serial

class SerialClient:
    def __init__(self, port, baud_rate):
        print("Starting serial...")
        self.serial = serial.Serial(port, baud_rate, timeout=0)
        time.sleep(2)
        if not self.serial.isOpen():
            print("Serial port is closed. Exiting..")
            exit(1)
        else:
            print("Successfully connected.")

    def send(self, lin_vel, ang_vel):
        self.serial.write(f"{lin_vel} {ang_vel}".encode())

    def read(self):
        return self.serial.readline().decode()