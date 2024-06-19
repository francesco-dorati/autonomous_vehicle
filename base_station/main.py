from enum import Enum

from manual_console import ManualConsole
from autonomous_console import AutonomousConsole

import socket

HOSTNAME = "172.20.10.7"
MAIN_PORT = 5500
MANUAL_PORT = 5501
AUTO_PORT = 5502


class Mode(Enum):
    NULL = 0
    MANUAL = 1
    AUTO = 2

class RemoteConsole:
    def __init__(self, hostname, main_port, port_auto, port_manual):
        self.hostname = hostname
        self.main_port = main_port
        self.port_auto = port_auto
        self.port_manual = port_manual

        self.main_socket = None
        self.mode = Mode.NULL

    def start(self):
        # try connection with main server
        print(f"Connecting to main server ({self.hostname}:{self.main_port})...")
        try:
            self.main_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.main_socket.connect((self.hostname, self.main_port))
            print("Connection established.")

        except socket.error:
            raise Exception("Socket error.")

        while True:
            if self.mode == Mode.NULL:

                self.mode = self._mode_menu()
                if self.mode == Mode.NULL:
                    self.main_socket.send("EXIT".encode())
                    self.main_socket.close()
                    return
                continue

            # elif self.mode == Mode.AUTO:
            #     a = AutonomousConsole(self.hostname, self.port_auto)
            #     a.run()
            #     self.mode = Mode.NULL
            #     continue

            elif self.mode == Mode.MANUAL:
                self.main_socket.send("MANUAL".encode())
                data = self.main_socket.recv(1024)
                if data.decode().strip()[0] != "OK":
                    print("Connection failed.")
                    continue
                m = ManualConsole(self.hostname, self.port_manual)
                m.run()
                self.mode = Mode.NULL
                continue
    
    def _mode_menu(self):
        print("\n\n\nROBOT CONTROLLER\n")
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
                return self.mode
            
            except ValueError:
                continue

if __name__ == "__main__":
    r = RemoteConsole(HOSTNAME, AUTO_PORT, MANUAL_PORT)
    r.start()

# commands
# rot <degrees>
# mv <distance> 

# spd <lin_speed> <angl_speed>
