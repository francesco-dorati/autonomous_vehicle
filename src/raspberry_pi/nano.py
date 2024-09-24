import serial

class NANO:
    def __init__(self):
        self.ser = serial.Serial('/dev/serial0', 9600)
    def send_power(self, pow_l, pow_r):
        self.ser.write(f'P {pow_l} {pow_r}\n'.encode())
    def close(self):
        self.send_power(0, 0)
        self.ser.close()
