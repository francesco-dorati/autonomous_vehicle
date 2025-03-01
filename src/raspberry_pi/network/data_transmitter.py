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
import zlib
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
        """
        Il messaggio include nell'header:
        * Magic number (4 byte)
        * Versione (2 byte)
        * Lunghezza del payload compresso (4 byte)
        * Grid size (2 byte)
        * Numero dei punti lidar (2 byte)

        Il payload è la concatenazione di:
        - I byte della griglia (occupancy grid, ottenuti con get_bytes())
        - I punti (ogni punto è 2 interi, 4 byte in totale per punto)?
        - La posizione (3 interi, 12 byte)
        """
        while self._running:
            try:
                if self._robot.get_control_type() != "off":
                    logger.debug("DATA TRANSMITTER start")

                    # Collect data from the robot
                    global_map, lidar_points, position = self._robot.get_data(DATA_SERVER_CONFIG.SIZE_MM)
                    
                    logger.debug("DATA TRANSMITTER got data")
                    # logger.debug(f"DATA TRANSMITTER lidar points: {lidar_points}")
                    
                    # occupancy grid
                    grid_size = global_map.get_grid_size()
                    real_grid_size = grid_size if global_map.is_set() else 0
                    grid_bytes = global_map.get_bytes()

                    logger.debug(f"DATA TRANSMITTER grid size: {grid_size}")

                    # lidar points
                    lidar_size = 0  # size in number of points
                    lidar_bytes = b""
                    if lidar_points:
                        lidar_size = len(lidar_points)
                        # logger.debug(f"packing lidar points: {lidar_points}")
                        for x, y in lidar_points:
                            lidar_bytes += struct.pack("<hh", x, y)

                    logger.debug(f"DATA TRANSMITTER local points: {lidar_size}")
                    
                    # position
                    position_valid = position is not None
                    position_bytes = b""
                    if position_valid:
                        position_bytes = struct.pack("3i", position.x, position.y, position.th)

                    logger.debug(f"DATA TRANSMITTER position: {position_valid}")

                    # compose payload
                    payload = grid_bytes + lidar_bytes + position_bytes
                    checksum = zlib.crc32(payload)
                    compressed_payload = zlib.compress(payload)


                    logger.debug(f"DATA TRANSMITTER compressed payload size: {len(compressed_payload)}, checksum {checksum}")
                    logger.debug(f"DATA TRANSMITTER normal payload size: {len(payload)}")

                    # Header
                    # Formato: "4sHIHH"
                    #   4s  -> Magic number (4 byte)
                    #   H   -> Versione (2 byte)
                    #   I   -> Lunghezza del payload compresso (4 byte)
                    #   H   -> Grid width (2 byte)
                    #   H   -> Numero dei punti (2 byte)
                    #   ?   -> Posizione Valida (1 byte)
                    header = struct.pack("4sHIHHH?", b'RBT1', 1, len(compressed_payload),
                                        grid_size, real_grid_size, lidar_size, position_valid)
                    
                    logger.debug(f"DATA TRANSMITTER header bytes: {header}")

                    self._socket.sendall(header + compressed_payload)

                    logger.debug(f"DATA TRANSMITTER data sent.")

            except BrokenPipeError:
                logger.error("Server connection lost.")
                self.stop()
                return
            
            except Exception as e:
                logger.error(f"DataTransmitter error: {e}")

            time.sleep(DATA_SERVER_CONFIG.INTERVAL)