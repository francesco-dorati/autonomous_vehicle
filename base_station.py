import socket
import time
import keyboard
import json
from enum import Enum

HOSTNAME = "172.20.10.7"
PORT = 0
MANUAL_FREQ = 5
MANUAL_TAO = 1/MANUAL_FREQ

class Mode(Enum):
    NULL = 0
    AUTO = 1
    MANUAL = 2

class RemoteConsole:
    def __init__(self, hostname, port):
        try:
            print("Establishing socket connection...")
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.connect((hostname, port))
            print(f"Connected successfully to socket {hostname}:{port}")
        except:
            print("Connection failed. Exiting...")
            exit(1)

        self.mode = Mode.NULL

    def start():
        while True:
            if mode == Mode.NULL:
                mode = _mode_menu()
                self.socket.send(mode.name.encode("utf-8"))
                continue

            elif mode == Mode.AUTO:
                _autonomous_console()
                self.mode = Mode.NULL
                self.socket.send("EXIT".encode())
                continue

            elif mode == Mode.MANUAL:
                _manual_console()
                self.mode = Mode.NULL
                self.socket.send("EXIT".encode())
                continue
    
    def _mode_menu():
        print("\n\nROBOT CONTROLLER\n")
        print("Select a mode: ")
        for m in Mode:
            print(f"{m.value}. {m.name}")
        print("0. Exit")
        print()

        while True:
            mode = input("> ")
            if mode == "0":
                self.socket.close()
                exit()
            try:
                return Mode(int(mode))
            except ValueError:
                continue

    def _autonomous_console():
        print("\nAUTONOMOUS CONSOLE\n")
        while True:
            command = input("AUTO> ").split()

            if len(command) == 0:
                continue

            if command[0] == "exit" or command[0] == "EXIT":
                return

            if command[0] == "mv" or command[0] == "m": # mv 100 (cm)
                try:
                    dist_cm = int(command[1])
                    self.socket.send(f"m {dist_cm}".encode())
                    ack = self.socket.recv(1024).decode('utf-8')
                    print(ack)
                finally:
                    continue
  
            elif command[0] == "rot" or command[0] == "r": # rot 90 (deg)
                try:
                    angle_deg = int(command[1])
                    self.socket.send(f"r {angle_deg}".encode())
                    ack = self.socket.recv(1024).decode('utf-8')
                    print(ack)
                finally:
                    continue

            else: 
                continue

    def _manual_console():
        print("\nMANUAL CONSOLE\n")
        while True:
            start_time = time.time()
            input_buffer = []

            if keyboard.is_pressed('esc') or keyboard.is_pressed("space"):
                return
            
            # read keypresses
            if keyboard.is_pressed('w'):
                input_buffer.append("f") # forward
            if keyboard.is_pressed("s"):
                input_buffer.append("b") # backward
            if keyboard.is_pressed("a"):
                input_buffer.append("l") # left
            if keyboard.is_pressed("d"):
                input_buffer.append("r") # right
            
            # buffer processing
            if "f" in input_buffer and "b" in input_buffer:
                input_buffer.remove("f")
                input_buffer.remove("b")
            if "l" in input_buffer and "r" in input_buffer:
                input_buffer.remove("l")
                input_buffer.remove("r")
            
            # send buffer
            self.socket.send(json.dumps(input_buffer).encode('utf-8'))

            # wait until next iteration
            dt = time.time() - start_time
            if dt < MANUAL_TAO:
                time.sleep(MANUAL_TAO - dt) 




def main():

    r = RemoteConsole()
    r.start()


    
if __name__ == "__main__":
    main()

# commands
# rot <degrees>
# mv <distance> 

# spd <lin_speed> <angl_speed>
