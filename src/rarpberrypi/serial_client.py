import time
import serial

class SerialClient:
    def __init__(self, port, baud_rate):
        print("[SERIAL] Starting serial...")
        self.serial = serial.Serial(port, baud_rate, timeout=1)
        time.sleep(2)
        if not self.serial.isOpen():
            print("[SERIAL] Serial port is closed. Exiting...")
            exit(1)
        else:
            print("[SERIAL] Successfully connected.\n")
    
    def start(self):
        self.serial.write("S".encode())
        print("[SERIAL] Starting controller...")
        data = self.serial.readline().decode().strip()
        while data != "OK":
            time.sleep(.01)
            data = self.serial.readline().decode().strip()
        print("[SERIAL] Controller started.\n")

    def stop(self):
        self.serial.write("E".encode())
        print("[SERIAL] Stopping controller...")
        data = self.serial.readline().decode().strip()
        while data != "OK":
            time.sleep(.01)
            data = self.serial.readline().decode().strip()
        print("[SERIAL] Controller stopped.\n")
            

    def send(self, lin_vel, ang_vel):
        data = f"V {lin_vel} {ang_vel}\n"
        print(f"\n[SERIAL] Sending: {data.strip()}")
        self.serial.write(data.encode())

    def read(self):
        data = self.serial.readline().decode().strip()
        while data == "":
            data = self.serial.readline().decode().strip()
        print(f"[SERIAL] Received: {data}")
        return data