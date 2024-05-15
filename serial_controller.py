import serial
import time

serial_port = '/dev/cu.usbserial-110'
baud_rate = 9600

s = serial.Serial(serial_port, baud_rate)
# while True:
#     input_data = input("Enter data: ")
#     try:
#         lin_vel, ang_vel = map(int, input_data.split(" "))
#     except:
#         print("Invalid input")

#     s.write(f"{lin_vel} {ang_vel}\n".encode())
s.write(f"0 0\n".encode())
print("Starting...")
time.sleep(5)
s.write(f"3 0\n".encode())
print("Sent 3 0")
time.sleep(5)
s.write(f"0 10\n".encode())
print("Sent 0 10")
time.sleep(5)
s.write(f"0 -10\n".encode())
print("Sent 0 -10")
time.sleep(5)
s.write(f"-3 0\n".encode())
print("Sent -3 0")
time.sleep(5)
s.write(f"0 0\n".encode())
print("END")
    