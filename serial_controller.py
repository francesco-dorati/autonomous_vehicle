import serial
import time
import keyboard
from enum import Enum

serial_port = '/dev/cu.usbserial-110'
baud_rate = 9600

MANUAL_FREQ = 5

class Mode(Enum):
    AUTO = 1
    MANUAL = 2

def main():
    print("Starting serial...")
    s = serial.Serial(serial_port, baud_rate)
    print("Connected.\n")
    
    mode = 0
    while True:
        if mode == 0: # CHANGE MODE
            mode = mode_menu()
            s.write(f"{mode.name}\n".encode())
            continue

        if mode == Mode.AUTO: # AUTO MODE
            command = input("AUTO> ").split()
            if command[0] == "exit":
                mode = 0
                continue

            elif command[0] == "mv":
                try:
                    dist_cm = int(command[1])
                    s.write(f"mv {dist_cm}\n".encode())
                finally:
                    continue
  
            elif command[0] == "rot":
                try:
                    angle_deg = int(command[1])
                    s.write(f"rot {angle_deg}\n".encode())
                finally:
                    continue
            else: 
                continue
            

        elif mode == Mode.MANUAL: # MANUAL MODE
            while True:
                kbd_buffer = []
                # keyboard reading
                if keyboard.is_pressed("esc") or keyboard.is_pressed("space"):
                    mode = 0
                    break
                if keyboard.is_pressed("w"):
                    kbd_buffer.append("f") # forward
                if keyboard.is_pressed("s"):
                    kbd_buffer.append("b") # backward
                if keyboard.is_pressed("a"):
                    kbd_buffer.append("l") # left
                if keyboard.is_pressed("d"):
                    kbd_buffer.append("r") # right
                
                # buffer processing
                if "f" in kbd_buffer and "b" in kbd_buffer:
                    kbd_buffer.remove("f")
                    kbd_buffer.remove("b")
                if "l" in kbd_buffer and "r" in kbd_buffer:
                    kbd_buffer.remove("l")
                    kbd_buffer.remove("r")
                
                command = "".join(kbd_buffer)
                s.write(f"{command}\n".encode())

                

def mode_menu():
    print("Mode: ")
    for m in Mode:
        print(f"{m.value}. {m.name}")
    print("0. Exit")
    print()

    while True:
        mode = input("> ")
        if mode == "0":
            exit()
        try:
            return Mode(int(mode))
        except ValueError:
            continue


def manual_controller():
    while True:
        input_data = input("")
        try:
            lin_vel, ang_vel = map(int, input_data.split(" "))
        except:
            print("Invalid input")

        s.write(f"{lin_vel} {ang_vel}\n".encode())
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
    
if __name__ == "__main__":
    main()

# commands
# rot <degrees>
# mv <distance> 

# spd <lin_speed> <angl_speed>