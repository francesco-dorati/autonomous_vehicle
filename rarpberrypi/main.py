import time
import socket
from enum import Enum

# from socket_server import TCPServer, ManualServer
from serial_client import SerialClient
#from autonomous_controller import AutonomousController
# from manual_controller import ManualController

SERIAL_PORT = '/dev/ttyUSB0'
SERIAL_RATE = 115200


# MAIN SERVER COMMANDS:
# PING  ->  OK
# MANUAL START -> OK <port>
# MANUAL STOP -> OK
# DATA START -> OK <port>
# DATA STOP -> OK
# CAMERA START -> OK <port>
# CAMERA STOP -> OK
# SHUT DOWN -> OK

from manual_server import ManualServer
from data_server import DataServer
from camera_server import CameraServer

class MainServer:
    def __init__(self):
        self.HOSTNAME = '172.20.10.7'
        self.MAIN_PORT = 5500
        self.MANUAL_PORT = 5501
        self.DATA_PORT = 5502
        self.CAMERA_PORT = 5502

        self.serial = SerialClient(SERIAL_PORT, SERIAL_RATE)

        self.main_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.main_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.main_socket.bind((self.HOSTNAME, self.MAIN_PORT))
        self.main_socket.listen(1)
        self.main_connection = None
        self.main_address = None
        print(f"[MAIN SERVER] Ready. Listening on {self.HOSTNAME}:{self.MAIN_PORT}...")

        self.manual_server = None
        self.data_server = None
        self.camera_server = None
    
    def loop(self):
        while True:
            if self.main_connection is None: # WAIT CONNECTION
                self.main_connection, self.main_address = self.main_socket.accept()
                print(f"[MAIN SERVER] Connected to {self.main_address[0]}.\n")

            else:   # CONNECTED
                try:
                    data = self.main_connection.recv(1024)
                    if not data:
                        continue

                    command = data.decode().strip().split()

                    if command[0] == "EXIT":
                        self.main_connection.close()
                        self.main_connection = None
                        print(f"[MAIN SERVER] Client disconnected.")
                        break

                    elif command[0] == "PING":
                        self.main_connection.send("OK".encode())

                    elif command[0] == "MANUAL":
                        if command[1] == "START":
                            if self.manual_server is None:
                                print(f"[MAIN SERVER] Starting manual controller...")
                                self.manual_server = ManualServer(self.HOSTNAME, self.MANUAL_PORT, self.main_address[0])
                                self.manual_server.start()
                                self.main_connection.send(f"OK {self.MANUAL_PORT}".encode())
                                print("[MAIN SERVER] Manual server started succesfully.\n")
                            else:
                                self.main_connection.send("KO".encode())
                                print(f"[MAIN SERVER] Manual controller is already running.")
                        elif command[1] == "STOP":
                            if self.manual_server is not None:
                                print(f"[MAIN SERVER] Stopping manual controller...")
                                self.manual_server.stop()
                                self.manual_server = None
                            else:
                                self.main_connection.send("KO".encode())
                                print(f"[MAIN SERVER] Manual controller is not running.")

                    elif command[0] == "DATA":
                        if command[1] == "START":
                            if self.data_server is None:
                                print(f"[MAIN SERVER] Starting data server...")
                                self.data_server = DataServer(self.HOSTNAME, self.DATA_PORT, self.serial)
                                self.data_server.start()
                                self.main_connection.send(f"OK {self.DATA_PORT}".encode())
                                print("[MAIN SERVER] Data server started succesfully.\n")
                            else:
                                self.main_connection.send("KO".encode())
                                print(f"[MAIN SERVER] Data server is already running.")
                        elif command[1] == "STOP":
                            if self.data_server is not None:
                                print(f"[MAIN SERVER] Stopping data server...")
                                self.data_server.stop()
                                self.data_server = None
                            else:
                                self.main_connection.send("KO".encode)
                                print(f"[MAIN SERVER] Data server is not running.")

                    elif command[0] == "CAMERA":
                        if command[1] == "START":
                            if self.camera_server is None:
                                print(f"[MAIN SERVER] Starting camera server...")
                                self.camera_server = CameraServer(self.HOSTNAME, self.CAMERA_PORT, 30)
                                self.camera_server.start()
                                self.main_connection.send(f"OK {self.CAMERA_PORT}".encode())
                                print("[MAIN SERVER] Camera server started succesfully.\n")
                            else:
                                self.main_connection.send("KO".encode())
                                print(f"[MAIN SERVER] Camera server is already running.")
                        elif command[1] == "STOP":
                            if self.camera_server is not None:
                                print(f"[MAIN SERVER] Stopping camera server...")
                                self.camera_server.stop()
                                self.camera_server = None
                            else:
                                self.main_connection.send("KO".encode)
                                print(f"[MAIN SERVER] Camera server is not running.")
                
                    elif command[0] == "SHUTDOWN":
                        pass

                except BrokenPipeError:
                    print(f"[MAIN SERVER] Connection closed by the client: {self.main_address[0]}")
                    self.main_connection.close()
                    self.main_connection = None
                    self.main_address = None

                except socket.error as e:
                    self.main_connection.close()
                    raise Exception(f"[MAIN SERVER] Connection closed by error {e}.")

                except KeyboardInterrupt:
                    self.main_connection.close()
                    self.main_socket.shutdown(socket.SHUT_RDWR)
                    exit(0)
            
if __name__ == "__main__":
    main_server = MainServer()
    main_server.loop()

# def main():
#     # start serial client
#     ser = SerialClient(SERIAL_PORT, SERIAL_RATE)

#     # start main socket
#     main_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
#     main_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
#     main_socket.bind((HOSTNAME, MAIN_SOCKET_PORT))
#     main_socket.listen(1)

#     main_connection = None
#     main_addr = None

#     print(f"[MAIN SERVER] Ready. Listening on {HOSTNAME}:{MAIN_SOCKET_PORT}...")

#     # MAIN SERVER LOOP (request - response)
#     while True:
#         if main_connection is not None:
#             try:
#                 # read command
#                 data = main_connection.recv(1024)
#                 if not data:
#                     continue

#                 command = data.decode().strip().split()
                
#                 if command[0] == "EXIT":
#                     main_connection.close()
#                     print(f"[MAIN SERVER] Client disconnected.")
#                     break

#                 elif command[0] == "PING":
#                     main_connection.send("OK".encode())

#                 elif command[0] == "MANUAL":
#                     if len(command) != 2:
#                         raise Exception("[MAIN SERVER] Invalid command.")
#                     if command[1] == "START":
#                         # start manual controller
#                         print(f"[MAIN SERVER] Starting manual controller...")
#                         manual_thread = ManualController(HOSTNAME, MANUAL_SOCKET_PORT, ser)
#                         manual_thread.start()
#                         main_connection.send(f"OK {MANUAL_SOCKET_PORT}".encode())
#                         print("[MAIN SERVER] Manual server started succesfully.\n")

#                         # manual server

#             except BrokenPipeError:
#                 print(f"[MAIN SERVER] Connection closed by the client: {main_addr}")
#                 main_socket.close()
#                 main_socket = None
#                 main_addr = None
#                 mode = mode.WAIT_CONNECTION

#             except socket.error as e:
#                 main_socket.close()
#                 raise Exception(f"[MAIN SERVER] Connection closed by error {e}.")

#             except KeyboardInterrupt:
#                 main_socket.close()
#                 main_socket = None
#                 main_addr = None
#                 exit(0)
#         else:
#             # accept new connection
#             main_connection, main_addr = main_socket.accept() # blocking
#             print(f"[MAIN SERVER] Connected to {main_addr}.\n")
#             mode = Mode.CONNECTED


#     # auto_socket = TCPServer(HOSTNAME, AUTO_SOCKET_PORT)
#     # manual_socket = ManualServer(HOSTNAME, MANUAL_SOCKET_PORT)

#     # auto_socket.start()
#     # manual_socket.start()

#     # mode = Mode.WAIT_CONNECTION
#     # while True:
#     #     if mode == Mode.WAIT_CONNECTION:
#     #         # Check connection to main


#     #         # Check connections
#     #         if auto_socket.connected and manual_socket.connected:
#     #             auto_socket.end_connection()
#     #             manual_socket.end_connection()

#     #         elif auto_socket.connected:
#     #             mode = Mode.AUTO
#     #             manual_socket.block()

#     #         elif manual_socket.connected:
#     #             mode = Mode.MANUAL
#     #             auto_socket.block()

#     #     elif mode == Mode.AUTO:
#     #         auto_controller = AutonomousController(auto_socket, ser)
#     #         auto_controller.loop()
#     #         auto_socket.end_connection()
#     #         ser.stop()

#     #         mode = Mode.WAIT_CONNECTION
#     #         manual_socket.resume()
#     #         continue

#     #     elif mode == Mode.MANUAL:
#     #         manual_thread = ManualController(manual_socket, ser)
#     #         manual_thread.loop()
#     #         manual_socket.end_connection()
#     #         ser.stop()

#     #         mode = Mode.WAIT_CONNECTION
#     #         auto_socket.resume()
#     #         continue

#     #     time.sleep(0.5)

    
# if __name__ == "__main__":
#     main()

