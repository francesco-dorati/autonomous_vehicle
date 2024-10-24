# Import necessary libraries
import serial
import time
import threading
import struct
from enum import Enum


class Lidar:
    PORT = '/dev/ttyUSB0'  
    BAUD_RATE = 460800 
    TIMEOUT = 0.05
    START_COMMAND = b'\xA5\x20'
    START_RESPONSE_DESCRIPTOR = b"\xA5\x5A\x05\x00\x00\x40\x81"
    STOP_COMMAND = b'\xA5\x25'
    HEALTH_COMMAND = b'\xA5\x52'
    HEALTH_RESPONSE_DESCRIPTOR = b"\xA5\x5A\x03\x00\x00\x00\x06"


    def __init__(self):
        self.ser = serial.Serial(self.PORT, self.BAUD_RATE, timeout=self.TIMEOUT)
        self.scanning = False
        self.scan = []
        self.get_health()

    
    def get_health(self):
        self.ser.write(self.HEALTH_COMMAND)
        response_descriptor = self.ser.read(7)
        if response_descriptor != self.START_RESPONSE_DESCRIPTOR:
            print()
            self.scanning = False
            return

        data = self.ser.read(3)
        status = data[0].decode()
        print(status)

        
    
    def start(self):
        # start scanning thread
        self.ser.write(self.START_COMMAND)
        response_descriptor = self.ser.read(7)
        if response_descriptor != self.START_RESPONSE_DESCRIPTOR:
            print(f"Wrong response descriptor: {response_descriptor}")
            self.scanning = False
            return
        
        self.scanning = True
        self.scan_thread = threading.Thread(target=self.scan)
        self.scan_thread.start()

    def scan(self): # THREAD
        while self.scanning:
            data = self.ser.read(5)
            if len(data) != 5:
                print(f"LIDAR ignored data: {data}")
                continue
            
            print(data)
            self.unpack_data(data)
    
    def unpack_data(self, data):
        s = (data[0] & 0b00000001)  # Start flag (0-1)
        ns = (data[0] & 0b00000010) >> 1  # Inverted start flag (1-2)
        quality = (data[0] >> 2) & 0b00111111  # Quality (2-8)
        c = (data[0] & 0b01000000) >> 6  # Check bit (8-9)
        angle_q6_low = data[1]  # angle_q6[6:0] (9-16)
        angle_q6_high = data[2]  # angle_q6[14:7] (16-24)
        distance_low = data[3]  # distance_q2[7:0] (24-32)
        distance_high = data[4]  # distance_q2[15:8] (32-40)

        # Calculate actual angle and distance
        angle_q6 = (angle_q6_high << 7) | angle_q6_low  # Combine high and low bytes for angle
        angle = angle_q6 / 64.0  # Convert to degrees
        distance_q2 = (distance_high << 8) | distance_low  # Combine high and low bytes for distance
        distance = distance_q2 / 4.0  # Convert to mm

        return s
    

    def stop(self):
        self.ser.write(self.STOP_COMMAND)
        self.scanning = False
    



# Define the serial port and baud rate
PORT = '/dev/ttyUSB0'  # Replace with your RPLIDAR port
BAUD_RATE = 460800  # Adjust if necessary

# Function to communicate with RPLIDAR
def communicate_with_rplidar():
    with serial.Serial(PORT, BAUD_RATE, timeout=1) as ser:
        try:
        # Initialize serial connection
            print("Serial connection established.")
            
            # Send command to start the motor (0xA5 for starting motor)
            start_motor_cmd = b'\xA5\x20'  # Modify if needed based on RPLIDAR commands
            ser.write(start_motor_cmd)
            print("Sent command to start motor.")

            # Give some time for the motor to start
            time.sleep(2)

            # Read data continuously
            while True:
                if ser.in_waiting > 0:
                    data = ser.read(ser.in_waiting)  # Read available bytes
                    print(f"Received data: {data.hex()}")  # Print data in hexadecimal format
                else:
                    print("No data available.")
                time.sleep(0.1)

        except KeyboardInterrupt:
            start_motor_cmd = b'\xA5\x25'  # Modify if needed based on RPLIDAR commands
            ser.write(start_motor_cmd)
            print("Sent command to stop motor.")

# Run the function
communicate_with_rplidar()