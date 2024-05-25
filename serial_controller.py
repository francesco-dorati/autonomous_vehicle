import serial
import socket
import time
import keyboard
from enum import Enum

SERIAL_PORT = '/dev/ttyUSB0'
SERIAL_RATE = 9600

SOCKET_PORT = 5500
MANUAL_FREQ = 5


class Mode(Enum):
    NULL = 0
    AUTO = 1
    MANUAL = 2

class SocketServer:
    def __init__(self, port):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        hostname = socket.gethostname()
        self.socket.bind((hostname, port))
        self.socket.listen(1)
        print(f"Server listening on {hostname}:{port}")
    
    def await_connection(self):
        while True:
            client_socket, addr = self.socket.accept()
            print("Established a connection with", addr)
            return client_socket
        

class SerialClient:
    def __init__(self, port, baud_rate):
        print("Starting serial...")
        self.serial = serial.Serial(port, baud_rate, timeout=0)
        time.sleep(2)
        if not self.serial.isOpen():
            print("Serial port is closed. Exiting..")
            exit(1)
        else:
            print("Successfully connected.")
        

def command_handler(command):


def main():
    ser = SerialClient(SERIAL_PORT, SERIAL_RATE)

    socket_server = SocketServer(SOCKET_PORT)
    socket_connection = socket_server.await_connection()

    # start serial
    # wait for socket connection

    # recv command
    # send to serial
    # wait for serial response
    # send response to socket
    


      
    
    mode = 0
    while True:
        if mode == 0: # CHANGE MODE
            
            mode = mode_menu()

            s.flushInput()
            s.write(f"{mode.name}\n".encode("utf-8"))
            # response = ser.readline().decode().strip()  # Read response from Arduino
            # print("Response from Arduino:", response)
            continue

        if mode == Mode.AUTO: # AUTO MODE
            command = input("AUTO> ").split()
            if len(command) == 0:
                continue

            elif command[0] == "exit":
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
            

        elif mode == Mode.MANUAL: # MANUAL MODE
            print("MANUAL MODE")
            while True:
                t_start = time.time()
                kbd_buffer = []
                # keyboard reading
                keyboard.wait('a')
                print('space')
                # if keyboard.is_pressed('^[') or keyboard.is_pressed("space"):
                #     mode = 0
                #     s.write("exit\n".encode())
                #     break

                # if keyboard.is_pressed('w'):
                #     kbd_buffer.append("f") # forward
                # if keyboard.is_pressed("s"):
                #     kbd_buffer.append("b") # backward
                # if keyboard.is_pressed("a"):
                #     kbd_buffer.append("l") # left
                # if keyboard.is_pressed("d"):
                #     kbd_buffer.append("r") # right
                
                # # buffer processing
                # if "f" in kbd_buffer and "b" in kbd_buffer:
                #     kbd_buffer.remove("f")
                #     kbd_buffer.remove("b")
                # if "l" in kbd_buffer and "r" in kbd_buffer:
                #     kbd_buffer.remove("l")
                #     kbd_buffer.remove("r")
                
                # command = "".join(kbd_buffer)
                # print(kbd_buffer)
                # s.write(f"{command}\n".encode())

                # dt = time.time() - t_start
                # if dt < (1/MANUAL_FREQ):
                #     time.sleep((1/MANUAL_FREQ) - dt) 


def mode_menu():
    print("\n\nROBOT CONTROLLER\n")
    print("Select a mode: ")
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


    
if __name__ == "__main__":
    main()

# commands
# rot <degrees>
# mv <distance> 

# spd <lin_speed> <angl_speed>
