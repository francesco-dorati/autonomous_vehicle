import time
import serial

class SerialClient:
    def __init__(self, port, baud_rate):
        print("[SERIAL] Starting serial...")
        self.serial = serial.Serial(port, baud_rate, timeout=0)
        time.sleep(2)
        if not self.serial.isOpen():
            print("[SERIAL] Serial port is closed. Exiting...")
            exit(1)
        else:
            print("[SERIAL] Successfully connected.\n")

    def send(self, lin_vel, ang_vel):
        self.serial.write(f"{lin_vel} {ang_vel}\n".encode())

    def read(self):
        data = self.serial.readline().decode()
        print(f"[SERIAL] {data.strip()}")
        return data