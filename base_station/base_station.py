
import time
import keyboard
import json
import socket
from enum import Enum

HOSTNAME = "172.20.10.7"
MANUAL_PORT = 5500
AUTO_PORT = 5501
MANUAL_FREQ = 5
MANUAL_TAO = 1/MANUAL_FREQ

class Mode(Enum):
    NULL = 0
    AUTO = 1
    MANUAL = 2

class RemoteConsole:
    def __init__(self, hostname, port_auto, port_manual):
        print("OK")
        self.hostname = hostname
        self.port_auto = port_auto
        self.port_manual = port_manual

        self.mode = Mode.NULL

    def start(self):
        while True:
            if self.mode == Mode.NULL:
                self.mode = self._mode_menu()
                # self.socket.send(self.mode.name.encode("utf-8"))
                continue

            elif self.mode == Mode.AUTO:
                self._autonomous_console()
                self.mode = Mode.NULL
                self.socket.send("EXIT".encode())
                continue

            elif self.mode == Mode.MANUAL:
                self._manual_console()
                self.mode = Mode.NULL
                # self.socket.send("EXIT".encode())
                continue
    
    def _mode_menu(self):
        print("\n\nROBOT CONTROLLER\n")
        print("Select a mode: ")
        for m in Mode:
            if m.value == 0:
                continue
            print(f"{m.value}. {m.name}")
        print("0. Exit")
        print()

        while True:
            try:
                self.mode = Mode(int(input("> ")))
                if self.mode == Mode.NULL:
                    exit(0)
                return self.mode
            
            except ValueError:
                continue

    def _autonomous_console(self):
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

    def _manual_console(self):
        print("\nMANUAL CONSOLE\n\n\n")
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            
        except:
            raise Exception("Socket connection failed.")

        while True:
            start_time = time.time()
            input_buffer = []

            if keyboard.is_pressed('esc') or keyboard.is_pressed("space"):
                sock.sendto("EXIT".encode('utf-8'), (self.hostname, self.port_manual))
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
            arrow_up = '\u2191'
            arrow_down = '\u2193'
            arrow_left = '\u2190'
            arrow_right = '\u2192'
            arrows = {
                "f": arrow_up,
                "b": arrow_down,
                "l": arrow_left,
                "r": arrow_right
            }
            input_arrows = ''.join(arrows[key] for key in input_buffer)
            print('\r' + ' ' * 80 + '\r ', end='')
            print('\x1b[A' + '\r' + ' ' * 80 + '\r', end='')
            print('\x1b[A' + '\r' + ' ' * 80 + '\r', end='')
            print(f'    {input_arrows}  \n')
            sock.sendto(json.dumps(input_buffer).encode('utf-8'), (self.hostname, self.port_manual))

            # wait until next iteration
            dt = time.time() - start_time
            if dt < MANUAL_TAO:
                time.sleep(MANUAL_TAO - dt) 

    
if __name__ == "__main__":
    print("OK")
    r = RemoteConsole(HOSTNAME, AUTO_PORT, MANUAL_PORT)
    r.start()

# commands
# rot <degrees>
# mv <distance> 

# spd <lin_speed> <angl_speed>
