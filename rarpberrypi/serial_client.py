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
    
    def start(self):
        self.serial.write("START\n".encode())
        print("[SERIAL] Starting controller...\n")
        data = self.serial.readline().decode().strip()
        if data == "OK":
            print("[SERIAL] Controller started.\n")
        else:
            print(f"[SERIAL] Received: {data}")
            print("[SERIAL] Controller failed to start. Exiting...")
            exit(1)
            

    def send(self, lin_vel, ang_vel):
        data = f"{lin_vel} {ang_vel}\n"
        print(f"[SERIAL] Sending: {data.strip()}")
        self.serial.write(data.encode())

    def read(self):
        data = self.serial.readline().decode()
        if data.strip() != "":
            print(f"[SERIAL] {data.strip()}")
        return data