import serial
import time
import keyboard
from enum import Enum

serial_port = '/dev/ttyAMA0'
baud_rate = 9600

MANUAL_FREQ = 5

class Mode(Enum):
    AUTO = 1
    MANUAL = 2

def main():
    print("Starting serial...")

    s = serial.Serial(serial_port, baud_rate, timeout=0)
    time.sleep(2)
    print("Connected.\n")

    if s.isOpen():
        print("Serial port is open")
    else:
        print("Serial port is not open")
    
    mode = 0
    while True:
        if mode == 0: # CHANGE MODE
            
            mode = mode_menu()
            print(mode.name)
            s.flush()
            s.write(f"{mode.name}\n".encode())
            # response = ser.readline().decode().strip()  # Read response from Arduino
            # print("Response from Arduino:", response)
            print("sent")
            continue

        if mode == Mode.AUTO: # AUTO MODE
            command = input("AUTO> ").split()
            if command[0] == "exit":
                mode = 0
                s.write("exit\n".encode())
                continue

            elif command[0] == "mv" or command[0] == "m":
                try:
                    dist_cm = int(command[1])
                    s.write(f"m {dist_cm}\n".encode())
                finally:
                    continue
  
            elif command[0] == "rot" or command[0] == "r":
                try:
                    angle_deg = int(command[1])
                    s.write(f"r {angle_deg}\n".encode())
                finally:
                    continue
            else: 
                continue
            

        elif mode == MANUAL: # MANUAL MODE
            print("MANUAL MODE")
            while True:
                t_start = time.time()
                kbd_buffer = []
                # keyboard reading
                if keyboard.is_pressed("esc") or keyboard.is_pressed("space"):
                    mode = 0
                    s.write("exit\n".encode())
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

                time.sleep(1/MANUAL_FREQ - (time.time() - t_start))


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
            print(Mode(int(mode)))
            return Mode(int(mode))
        except ValueError:
            continue


    
if __name__ == "__main__":
    main()

# commands
# rot <degrees>
# mv <distance> 

# spd <lin_speed> <angl_speed>
