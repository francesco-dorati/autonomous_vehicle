from enum import Enum

from manual_console import ManualConsole
from autonomous_console import AutonomousConsole

import socket
import time

HOSTNAME = "172.20.10.7"
MAIN_PORT = 5500
# MANUAL_PORT = 5501
# AUTO_PORT = 5502


class Status(Enum):
    NOT_CONNECTED = 0
    CONNECTED = 1

class Action(Enum):
    MANUAL = 1
    EXIT = 0
    EXIT_SHUTDOWN = -1
    RETRY_CONN = -2

class Mode(Enum):
    MANUAL = 1


class RemoteConsole:
    def __init__(self, hostname, port):
        self.hostname = hostname
        self.port = port
        # self.port_auto = port_auto
        # self.port_manual = port_manual

        self.main_socket = None
        self.status = Status.NOT_CONNECTED

    def start(self):
        # try connection
        if self._connect():
            self.status = Status.CONNECTED
        
        # print menu
        while True:
            action = self._main_menu()
            if action == Action.MANUAL:
                if self.status == Status.NOT_CONNECTED:
                    raise Exception("NOT CONNECTED")
                
                self.main_socket.send("MANUAL".encode())
                print("Waiting for response...")
                data = self.main_socket.recv(1024)
                try:
                    data = data.decode().strip().split()
                    if data[0] == "OK":
                        manual_port = int(data[1])
                    else:
                        continue
                except:
                    continue

                m = ManualConsole(self.hostname, manual_port)
                m.run()
                continue
            
            elif action == Action.EXIT:
                if self.start == Status.CONNECTED:
                    self.main_socket.send("EXIT".encode())
                    self.main_socket.close()

                print("\nExiting...")
                time.sleep(1)
                exit(0)

            elif action == Action.EXIT_SHUTDOWN:
                # check connection
                pass

            else:
                pass







        # while True:
        #     if self.status == Mode.NOT_CONNECTED:
        #         connected = self._connect_main()
        #         if connected:
        #             self.status = Status.CONNECTED
                

        #     elif self.status == Mode.CONNECTED:
        #         self.mode = self._mode_menu()
        #         if self.mode == Mode.NULL:
        #             self.main_socket.send("EXIT".encode())
        #             self.main_socket.close()
        #             return
        #         continue

            # elif self.mode == Mode.AUTO:
            #     a = AutonomousConsole(self.hostname, self.port_auto)
            #     a.run()
            #     self.mode = Mode.NULL
            #     continue

            # elif self.mode == Mode.MANUAL:
            #     self.main_socket.send("MANUAL".encode())
            #     data = self.main_socket.recv(1024)
            #     if data.decode().strip()[0] != "OK":
            #         print("Connection failed.")
            #         continue
            #     m = ManualConsole(self.hostname, self.port_manual)
            #     m.run()
            #     self.mode = Mode.NULL
            #     continue
    
    def _connect(self) -> bool:
        print(f"Connecting to the robot ({self.hostname}:{self.port})...")
        try:
            self.main_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.main_socket.settimeout(10)
            self.main_socket.connect((self.hostname, self.port))
            print("Connected succesfully to main server.")
            return True

        except (socket.timeout, ConnectionRefusedError):
            self.main_socket = None
            return False

        except socket.error:
            self.main_socket = None
            return False

    def _main_menu(self) -> Action:
        print("\n\n\n  ROBOT CONTROLLER\n")
        print(f" Status:  {'CONNECTED' if self.status == Status.CONNECTED else 'NOT CONNECTED'}\n")
        print(" Select an option: ")
        if self.status == Status.CONNECTED:
            print("  1)  Manual Mode")
            print("  2)  Autonomous Mode [TODO]")
            print("  3)  Settings [TODO]")
            print("  0)  Exit")
            print(" -1)  Exit & Shutdown")
        else:
            print(" 1)  Retry Connection")
            print(" 0)  Exit")
        print()
        

        while True:
            try:
                c = int(input("> "))
                if self.status == Status.CONNECTED:
                    if c == 1:
                        return Action.MANUAL
                    elif c == 2:
                        continue
                    elif c == 3:
                        continue
                    elif c == 0:
                        return Action.EXIT
                    elif c == -1:
                        return Action.EXIT_SHUTDOWN
                    else:
                        continue
                elif self.status == Status.NOT_CONNECTED:
                    if c == 1:
                        return Action.RETRY_CONN
                    elif c == 0:
                        return Action.EXIT
                    else:
                        continue

                return self.mode
            
            except ValueError:
                continue


if __name__ == "__main__":
    r = RemoteConsole(HOSTNAME, MAIN_PORT)
    r.start()

# commands
# rot <degrees>
# mv <distance> 

# spd <lin_speed> <angl_speed>
