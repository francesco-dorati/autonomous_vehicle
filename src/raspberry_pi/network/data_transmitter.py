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
                # Build the payload
                logger.info("Sending data...")
                payload = f"DATA\n{DATA_SERVER_CONFIG.SIZE_MM // ROBOT_CONFIG.GLOBAL_MAP_RESOLUTION}\n"
                logger.info("Sending data 1...")
                global_map, lidar_points, position = self._robot.get_data(DATA_SERVER_CONFIG.SIZE_MM)
                
                # Append GLOBAL MAP section
                logger.info("Sending adding globalmap...")
                payload += "GLOBAL_MAP\n"
                payload += f"{global_map.get_string()}\n"
                
                # Append LOCAL MAP (lidar points) section
                logger.info("Sending adding localmap...")
                payload += "LOCAL_MAP\n"
                logger.debug(f"lidar points: {lidar_points}")
                if len(lidar_points) > 0:
                    lidar_points_str = ";".join([f"{point[0]} {point[1]}" for point in lidar_points])
                    payload += f"{lidar_points_str}\n"
                else:
                    payload += "-\n"
                
                # Append POSITION section
                logger.info("Sending adding pos...")
                payload += "POSITION\n"
                logger.debug(f"position: {position}")
                if position:
                    payload += f"{position.x} {position.y} {position.th}\n"
                else:
                    payload += "-\n"
                
                # Send the complete payload via the established TCP connection
                self._socket.sendall(payload.encode())
            except Exception as e:
                logger.error(f"DataTransmitter error: {e}")
            time.sleep(DATA_SERVER_CONFIG.INTERVAL)
