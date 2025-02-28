import socket
import threading
import time
from typing import Tuple, List
import numpy as np
import struct
import zlib

class ClientConnection:
    """
    Handles low-level socket communication with the robot.
    This class encapsulates connection, disconnection, and sending commands.
    """
    def __init__(self, robot_address: str, port: int):
        self.robot_address = robot_address
        self.port = port
        self.connection = None

    def connect(self) -> bool:
        """Establishes a connection to the robot."""
        self.connection = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.connection.settimeout(3)  # Set an explicit timeout
        try:
            self.connection.connect((self.robot_address, self.port))
        except (socket.timeout, ConnectionRefusedError, OSError) as e:
            print(f"Connection failed: {e}")
            self.connection = None
            return False
        return True

    def disconnect(self):
        """Closes the connection if it exists."""
        if self.connection:
            self.connection.close()
            self.connection = None

    def ping(self):
        """ Ping
        Returns:
            ok: bool
            time_ms: int
            battery_V: float
            control_type: str

    """
        ping_time_ms: int = None
        battery_V: float = None
        control_type: str = None
        map_name: str = None
        try:
            t = time.time()
            self.connection.send("SYS PNG\n".encode())
            response = self.connection.recv(32)
            ping_time_ms = int((time.time() - t)*1000)
            split = response.decode().strip().split()
            if split[0] == "OK":
                battery_V = float(split[1])/1000
                control_type = None if split[2] == "-" else split[2]
                map_name = None if split[3] == "-" else split[3]
                return True, ping_time_ms, battery_V, control_type, map_name
            return False, None , None, None, None
        except Exception as e:
            print(f"Ping failed: {e}")
            return False, None , None, None, None

    def start_manual_control(self) -> Tuple[bool, int]:
        """Sends a command to the robot to start manual control."""
        if not self.connection:
            return False, 0
        try:
            self.connection.send("CTL MAN\n".encode())
            response = self.connection.recv(32).decode().strip()
            split = response.split(" ")
            if split[0] == "OK" and int(split[1]):
                return True, int(split[1])
        except Exception as e:
            print(f"Manual control failed: {e}")
        return False, 0

    def stop_control(self) -> None:
        """Sends a command to the robot to stop manual control."""
        if not self.connection:
            return
        
        try:
            print("Stopping control")
            self.connection.send("CTL STP\n".encode())
            response = self.connection.recv(32).decode().strip()
            return response == "OK"
        except Exception as e:
            print(f"Failed to stop control: {e}")
    
    # MAPPING COMMANDS
    def new_map(self, map_name: str) -> bool:
        """Sends a command to the robot to start a new map."""
        if not self.connection:
            return
        try:
            self.connection.send(f"MAP NEW {map_name}\n".encode())
            print("Sent new map command")
            response = self.connection.recv(32).decode().strip()
            print(response)
            return response == "OK"
        except Exception as e:
            print(f"Failed to send new map command: {e}")
            return False
        
    def discard_map(self) -> bool:
        """Sends a command to the robot to discard the current map."""
        if not self.connection:
            return
        try:
            self.connection.send("MAP DIS\n".encode())
            response = self.connection.recv(32).decode().strip()
            return response == "OK"
        except Exception as e:
            print(f"Failed to send discard map command: {e}")
            return False
    
    def save_map(self) -> bool:
        """Sends a command to the robot to save the current map."""
        if not self.connection:
            return
        try:
            self.connection.send("MAP SAV\n".encode())
            response = self.connection.recv(32).decode().strip()
            return response == "OK"
        except Exception as e:
            print(f"Failed to send save map command: {e}")
            return False

    def start_mapping(self) -> bool:
        """Sends a command to the robot to start mapping."""
        if not self.connection:
            return
        try:
            self.connection.send("MAP STR\n".encode())
            response = self.connection.recv(32).decode().strip()
            return response == "OK"
        except Exception as e:
            print(f"Failed to send start mapping command: {e}")
            return False
        
    def stop_mapping(self) -> bool:
        """Sends a command to the robot to stop mapping."""
        if not self.connection:
            return
        try:
            self.connection.send("MAP STP\n".encode())
            response = self.connection.recv(32).decode().strip()
            return response == "OK"
        except Exception as e:
            print(f"Failed to send stop mapping command: {e}")
            return False
