import socket
import threading
import time
import json  # (if needed)
from typing import List
from raspberry_pi.robot.robot import Robot
from raspberry_pi.utils.logger import get_logger
from raspberry_pi.data_structures.maps import OccupancyGrid
from raspberry_pi.data_structures.states import Position, CartPoint
from raspberry_pi.config import DATA_SERVER_CONFIG, ROBOT_CONFIG
import numpy as np
import struct

logger = get_logger(__name__)

class DataTransmitter:
    """
    TCP client that sends data to a remote server using a custom protocol.
    
    Data format:
        DATA\n
        <map_pixel_size>\n
        GLOBAL_MAP\n
        <global map separated by ';' (- for empty)>\n
        LOCAL_MAP\n
        <list of lidar map points (- for empty)>\n
        POSITION\n
        <x> <y> <theta>\n
        
    Notes:
      - The global map is an nxn occupancy grid.
      - The position is based on the global map (global coordinates). If no global map is available, 
        the position is represented as (0 0 0) (needed for orientation and the legend).
      - The local map is the list of points from the lidar (in frame coordinates).
    """
    def __init__(self, receiver_host: str, robot: Robot):
        self._robot: Robot = robot  # Robot instance from which to collect data
        self._receiver_host = receiver_host
        self._running = False
        self._thread = None
        # Create a TCP socket
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    
    def start(self):
        """Connects to the TCP server and starts the transmission loop."""
        try:
            self._socket.connect((self._receiver_host, DATA_SERVER_CONFIG.PORT))
            logger.info(f"Connected to server at {self._receiver_host}:{DATA_SERVER_CONFIG.PORT}")
        except Exception as e:
            logger.error(f"Failed to connect to server: {e}")
            return
        
        self._running = True
        self._thread = threading.Thread(target=self._transmit_loop, daemon=True)
        self._thread.start()
        logger.info("DataTransmitter started.")
    
    def stop(self):
        """Stops the transmission loop and closes the TCP connection."""
        self._running = False
        if self._thread:
            self._thread.join()
        try:
            self._socket.shutdown(socket.SHUT_RDWR)
        except Exception as e:
            logger.error(f"Error during socket shutdown: {e}")
        self._socket.close()
        logger.info("DataTransmitter stopped.")
    
    def _transmit_loop(self):
        """Continuously collects data from the robot and sends it to the server."""
        while self._running:
            try:
                logger.debug("DATA TRANSMITTER start")
                payload = bytearray(b"DATA\n")
                payload.extend(f"{DATA_SERVER_CONFIG.SIZE_MM // ROBOT_CONFIG.GLOBAL_MAP_RESOLUTION}\n".encode())

                # Collect data from the robot
                global_map, lidar_points, position = self._robot.get_data(DATA_SERVER_CONFIG.SIZE_MM)
                
                # Append global map data
                payload.extend(b"GLOBAL_MAP\n")
                global_map_bytes = global_map.get_bytes()
                payload.extend(struct.pack("I", len(global_map_bytes)))  # Add length of map data
                payload.extend(global_map_bytes)
                print(f"Sent global map of size: {len(global_map_bytes)} bytes")

                # Append lidar points data
                payload.extend(b"\nLOCAL_MAP\n")
                if lidar_points:
                    lidar_array = np.array(lidar_points, dtype=np.int16)
                    payload.extend(struct.pack("I", len(lidar_points)))  # Send number of points
                    payload.extend(lidar_array.tobytes())
                    print(f"Sent {len(lidar_points)} LIDAR points")
                else:
                    payload.extend(struct.pack("I", 0))  # No points
                    print("Sent empty LIDAR points")

                # Append position data
                payload.extend(b"\nPOSITION\n")
                if position:
                    payload.extend(f"{position.x} {position.y} {position.th}\n".encode())
                    print(f"Sent position: ({position.x}, {position.y}, {position.th})")
                else:
                    payload.extend(b"-\n")
                    print("Sent position: - (unknown)")

                # Send data with a size prefix
                total_length = struct.pack("I", len(payload))
                self._socket.sendall(total_length + payload)
                print(f"Total message size sent: {len(payload)} bytes")

            except BrokenPipeError:
                logger.error("Server connection lost.")
                self.stop()
                return
            except Exception as e:
                logger.error(f"DataTransmitter error: {e}")
            time.sleep(DATA_SERVER_CONFIG.INTERVAL)