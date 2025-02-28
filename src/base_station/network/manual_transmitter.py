import socket
import threading
import time
from typing import Tuple, List
import numpy as np
import struct
import zlib


MANUAL_INTERVAL = 0.1
class ManualTransmitter:
    """
    Handles a dedicated connection for manual control.
    This class connects to a given port, spawns a worker thread to handle communication,
    and uses an update callback to notify the application of received data.
    """
    def __init__(self, robot_address: str, port: int, input_handler):
        self.robot_address = robot_address
        self.port = port
        self.input_handler = input_handler 
        self.connection = None
        self.thread = None
        self.running = False

    def start(self):
        """Starts the manual server connection and worker thread."""
        self.connection = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.connection.settimeout(1)
        try:
            self.connection.connect((self.robot_address, self.port))
        except Exception as e:
            print("Error connecting to manual server:", e)
            return
        self.running = True
        self.thread = threading.Thread(target=self.worker, daemon=True)
        self.thread.start()

    def stop(self):
        """Stops the manual server and cleans up resources."""
        self.running = False
        if self.connection:
            self.connection.close()
        if self.thread:
            self.thread.join()

    def worker(self):
        """Worker thread that continuously reads from the manual connection."""
        while self.running:
            try:
                buffer = self.input_handler.get_buffer()
                data = f"{buffer['x']:.2f} {buffer['y']:.2f}\n"
                self.connection.send(data.encode())
            except Exception as e:
                print("Error in manual server worker:", e)
                self.connection.close()
                self.connection = None
                self.running = False
                break
            time.sleep(MANUAL_INTERVAL)

