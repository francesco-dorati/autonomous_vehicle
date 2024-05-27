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
        while data != "OK":
            time.sleep(.01)
            data = self.serial.readline().decode().strip()
        print("[SERIAL] Controller started.\n")

    # def stop(self):
    #     self.serial.write("STOP\n".encode())
    #     print("[SERIAL] Stopping controller...\n")
    #     data = self.serial.readline().decode().strip()
    #     while data != "OK":
    #         data = self.serial.readline().decode().strip()
    #         print(f"[SERIAL] Received: <{data}>")
    #         time.sleep(.01)
    #     print("[SERIAL] Controller started.\n")
            

    def send(self, lin_vel, ang_vel):
        data = f"{lin_vel} {ang_vel}\n"
        # print(f"[SERIAL] Sending: {data.strip()}")
        self.serial.write(data.encode('utf-8'))

    def read(self):
        data = self.serial.readline().decode().strip()
        if data != "":
            print(f"[SERIAL] Received: {data}")
        return data