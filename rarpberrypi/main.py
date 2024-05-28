import time
import socket
from enum import Enum

from socket_server import TCPServer, UDPServer
from serial_client import SerialClient
from autonomous_controller import AutonomousController
from manual_controller import ManualController

SERIAL_PORT = '/dev/ttyUSB0'
SERIAL_RATE = 115200

HOSTNAME = '172.20.10.7'
AUTO_SOCKET_PORT = 5500
MANUAL_SOCKET_PORT = 5501

class Mode(Enum):
    WAIT_CONNECTION = 0
    AUTO = 1
    MANUAL = 2

def main():
    # Start Sockets
    ser = SerialClient(SERIAL_PORT, SERIAL_RATE)
    auto_socket = TCPServer(HOSTNAME, AUTO_SOCKET_PORT)
    manual_socket = UDPServer(HOSTNAME, MANUAL_SOCKET_PORT)

    auto_socket.start()
    manual_socket.start()

    mode = Mode.WAIT_CONNECTION
    while True:
        if mode == Mode.WAIT_CONNECTION:
            if auto_socket.connected and manual_socket.connected:
                auto_socket.end_connection()
                manual_socket.end_connection()

            elif auto_socket.connected:
                mode = Mode.AUTO
            elif manual_socket.connected:
                mode = Mode.MANUAL

        elif mode == Mode.AUTO:
            auto_controller = AutonomousController(auto_socket, ser)
            auto_controller.loop()
            auto_socket.end_connection()
            mode = Mode.WAIT_CONNECTION
            continue

        elif mode == Mode.MANUAL:
            manual_controller = ManualController(manual_socket, ser)
            manual_controller.loop()
            manual_socket.end_connection()
            mode = Mode.WAIT_CONNECTION
            continue

        time.sleep(0.1)

    
if __name__ == "__main__":
    main()
