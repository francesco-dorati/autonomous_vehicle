import serial
import threading
import time

class NANO:
    _serial = None
    _request_lock = None

    @staticmethod
    def start():
        NANO._serial = serial.Serial('/dev/ttyAMA2', 9600, timeout=1)
        NANO._request_lock = threading.Lock()
        time.sleep(0.2)

    @staticmethod
    def stop():
        time.sleep(0.2)
        NANO._serial.close()
        NANO._serial = None
        NANO._request_lock = None


    @staticmethod
    def ping() -> bool:
        with NANO._request_lock:
            NANO._serial.write("P\n".encode())
            line = NANO._serial.readline()
        res = line.decode().strip()
        return res == 'P'
    
    @staticmethod
    def get_battery() -> int:
        with NANO._request_lock:
            NANO._serial.write("B\n".encode())
            line = NANO._serial.readline()
        if not line:
            return None
        battery_mv = int(line.strip())
        return battery_mv
    
    @staticmethod
    def start_sensors():
        NANO._serial.write("S1\n".encode())

    @staticmethod
    def stop_sensors():
        NANO._serial.write("S0\n".encode())

    @staticmethod
    def request_sensors() -> list:
        with NANO._request_lock:
            NANO._serial.write("SR\n".encode())
            line = NANO._serial.readline()
        if not line:
            return None
        fl, fr, rl, rr = map(int, line.decode().strip().split(" "))
        return [fl, fr, rl, rr]


