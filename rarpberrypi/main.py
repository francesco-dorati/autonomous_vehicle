import time
import socket
from enum import Enum

# from socket_server import TCPServer, ManualServer
from serial_client import SerialClient
#from autonomous_controller import AutonomousController
from manual_controller import ManualController

SERIAL_PORT = '/dev/ttyUSB0'
SERIAL_RATE = 115200

HOSTNAME = '172.20.10.7'
MAIN_SOCKET_PORT = 5500
MANUAL_SOCKET_PORT = 5501
AUTO_SOCKET_PORT = 5502

class Mode(Enum):
    WAIT_CONNECTION = 0     # wait for developer console to connect
    CONNECTED = 1                # settings, and servers handling

# import queue
# import socket
# import threading 
# class State:
#     def __init__(self):
#         self.vel = [.0, .0]
#         self.pos = [.0, .0, .0]
#         self.dist = [100, 100, 100, 100]

#     def update(self, vel, pos, dist):
#         self.vel = vel
#         self.pos = pos
#         self.dist = dist

# class MainController():
#     def __init__(self, hostname, port):
#         self.hostname = hostname
#         self.port = port

#         # START MAIN SOCKET
#         self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
#         self.socket.bind((self.hostname, self.port))
#         print(f"[MAIN SERVER] Listening on {self.hostname}:{self.port}")

#         self.connection = None
#         self.connection_addr = None

#         self.mode = Mode.WAIT_CONNECTION
#         self.state = State()

#         self.manual_thread = None
#         self.autonomous_thread = None
#         self.state_updater_thread = None
#         self.camera_controller = None

    
#     def run(self):
#         while True:
#             if self.mode == Mode.WAIT_CONNECTION:
#                 self.connection, self.connection_addr = self.socket.accept()
#                 self.mode = Mode.CONNECTED

#             elif self.mode == Mode.CONNECTED:
#                 try:
#                     data = self.connection.recv(1024)
#                     if not data:
#                         continue

#                     if data.decode().strip() == "EXIT":
#                         self.end_connection()
#                         print(f"[MAIN SERVER] Connection closed by client.")
#                         continue

#                     response = self.process_command(data.decode())
#                     self.connection.send(response.encode())

#                 except socket.error:
#                     self.end_connection()
#                     print(f"[MAIN SERVER] Connection closed by client.")
#                     continue
                
#     def end_connection(self):
#         self.mode = Mode.WAIT_CONNECTION
#         if self.connection is not None:
#             self.connection.close()
#         self.connection = None
    
#     def process_command(self, command):
#         command = command.strip().split()
#         if command[0] == "MANUAL":
#             if len(command) != 2:
#                 raise Exception("[MAIN SERVER] Invalid command.")
            
#             if command[1] == "START":
#                 if self.autonomous_controller is not None:
#                     return "KO Autonomous Controller is on."
                
#                 self.manual_thread = ManualController(HOSTNAME, MANUAL_SOCKET_PORT, self.state)
#                 self.manual_thread.start()
#                 return f"OK {self.manual_thread.port}"

#             elif command[1] == "STOP":
#                 if self.manual_thread is not None:
#                     self.manual_thread.stop()
#                     self.manual_thread = None
#                 return "OK"

                
#         # elif command[0] == "STATE":
#         #     if len(command) != 2:
#         #         raise Exception("[MAIN SERVER] Invalid command.")
            
#         #     if command[1] == "START":
#         #         self.state_publisher = StateUpdater(HOSTNAME, STATE_SOCKET_PORT, self.state)
#         #         self.state_publisher.start()
#         #         return f"OK {self.manual_thread.port}"
                
#         #     elif command[1] == "STOP":
#         #         if self.state_publisher is not None:
#         #             self.state_publisher.stop()
#         #             self.state_publisher = None
#         #         return "OK"
        
#         elif command[0] == "CAMERA":
#             if len(command) != 2:
#                 raise Exception("[MAIN SERVER] Invalid command.")
            
#             if command[1] == "START":
#                 pass
#             elif command[1] == "STATUS":
#                 pass
#             elif command[1] == "STOP":
#                 pass
        
#         elif command[0] == "SETTINGS":
#             pass


def main():
    # start serial client
    ser = SerialClient(SERIAL_PORT, SERIAL_RATE)

    # start main socket
    main_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    main_socket.bind((HOSTNAME, MAIN_SOCKET_PORT))
    main_socket.listen(1)
    main_connection = None
    main_addr = None

    print(f"[MAIN SERVER] Ready. Listening on {HOSTNAME}:{MAIN_SOCKET_PORT}...")

    # MAIN SERVER LOOP (request - response)
    while True:
        if main_connection is not None:
            try:
                # read command
                data = main_connection.recv(1024)
                if not data:
                    continue

                command = data.decode().strip().split()
                print(f"[MAIN SERVER] Received: \"{data.decode().strip()}\"")
                
                if command[0] == "EXIT":
                    main_connection.close()
                    print(f"[MAIN SERVER] Client disconnected.")
                    break

                elif command[0] == "MANUAL":
                    # start manual controller
                    print(f"[MAIN SERVER] Starting manual controller...")
                    manual_thread = ManualController(HOSTNAME, MANUAL_SOCKET_PORT, ser)
                    manual_thread.start()
                    main_socket.send(f"OK {MANUAL_SOCKET_PORT}".encode())
                    print("[MAIN SERVER] Manual server started succesfully.")

            except BrokenPipeError:
                print(f"[MAIN SERVER] Connection closed by the client: {main_addr}")
                main_socket.close()
                main_socket = None
                main_addr = None
                mode = mode.WAIT_CONNECTION

            except socket.error as e:
                main_socket.close()
                raise Exception(f"[MAIN SERVER] Connection closed by error {e}.")
        else:
            # accept new connection
            main_connection, main_addr = main_socket.accept() # blocking
            print(f"[MAIN SERVER] Connected to {main_addr}.\n")
            mode = Mode.CONNECTED


    # auto_socket = TCPServer(HOSTNAME, AUTO_SOCKET_PORT)
    # manual_socket = ManualServer(HOSTNAME, MANUAL_SOCKET_PORT)

    # auto_socket.start()
    # manual_socket.start()

    # mode = Mode.WAIT_CONNECTION
    # while True:
    #     if mode == Mode.WAIT_CONNECTION:
    #         # Check connection to main


    #         # Check connections
    #         if auto_socket.connected and manual_socket.connected:
    #             auto_socket.end_connection()
    #             manual_socket.end_connection()

    #         elif auto_socket.connected:
    #             mode = Mode.AUTO
    #             manual_socket.block()

    #         elif manual_socket.connected:
    #             mode = Mode.MANUAL
    #             auto_socket.block()

    #     elif mode == Mode.AUTO:
    #         auto_controller = AutonomousController(auto_socket, ser)
    #         auto_controller.loop()
    #         auto_socket.end_connection()
    #         ser.stop()

    #         mode = Mode.WAIT_CONNECTION
    #         manual_socket.resume()
    #         continue

    #     elif mode == Mode.MANUAL:
    #         manual_thread = ManualController(manual_socket, ser)
    #         manual_thread.loop()
    #         manual_socket.end_connection()
    #         ser.stop()

    #         mode = Mode.WAIT_CONNECTION
    #         auto_socket.resume()
    #         continue

    #     time.sleep(0.5)

    
if __name__ == "__main__":
    main()

