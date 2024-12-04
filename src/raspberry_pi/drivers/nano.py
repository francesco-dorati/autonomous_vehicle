import serial

class NANO:
    _serial = None

    @staticmethod
    def start():
        NANO._serial = serial.Serial('/dev/ttyAMA2', 9600, timeout=0.05)

    @staticmethod
    def stop():
        NANO._serial.close()
        NANO._serial = None

    @staticmethod
    def ping() -> bool:
        NANO._serial.write("B\n".encode())
        line = NANO._serial.readline()
        res = line.strip()
        return res == 'P'
    
    @staticmethod
    def get_battery() -> int:
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
    def get_distances() -> list:
        NANO._serial.write("SR\n".encode())
        line = NANO._serial.readline()
        if not line:
            return None
        fl, fr, rl, rr = map(int, line.strip().split(" "))
        return [fl, fr, rl, rr]


