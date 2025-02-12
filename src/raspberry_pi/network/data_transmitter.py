import socket
import threading
import time
import json
from typing import List
from raspberry_pi.utils.logger import get_logger
from raspberry_pi.data_structures.maps import OccupancyGrid
from raspberry_pi.data_structures.states import Position, CartPoint
from raspberry_pi.config import DATA_SERVER_CONFIG

logger = get_logger(__name__)

class DataTransmitter:
    """
    "
        DATA\n
        <map_pixel_size>\n
        GLOBAL_MAP\n
        <global map separated by ; (- for empty) >\n
        LOCAL_MAP\n
        <list of lidar map points (- for empty) >\n
        POSITION\n
        <x> <y> <theta>\n
    "   
    - nxn is tha size
    - position is based on the global map (global coordinates)
        if no global map -> position is (0 0 0)
        needed for the orientation and the legend
    - local map is the ist of points from the lidar
        in frame coordinates
    - global map is a nxn occupancy grid
    """
    def __init__(self, receiver_host: str, robot):
        self._robot = robot  # Robot instance from which to collect data
        self._receiver_host = receiver_host
        self._running = False
        self._thread = None
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # using UDP for simplicity

    def start(self):
        self._running = True
        self._thread = threading.Thread(target=self._transmit_loop, daemon=True)
        self._thread.start()
        logger.info(f"DataTransmitter started. Sending data to {self._receiver_host}:{DATA_SERVER_CONFIG.PORT}")

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join()
        self._socket.close()
        logger.info("DataTransmitter stopped.")

    def _transmit_loop(self):
        while self._running:
            try:
                payload = f"DATA\n{DATA_SERVER_CONFIG.SIZE}\n"
                global_map, lidar_points, position = self._robot.get_data(DATA_SERVER_CONFIG.SIZE)

                # GLOBAL MAP
                payload += "GLOBAL_MAP\n"
                if global_map:
                    payload += f"{global_map.to_string()}\n"
                else:
                    payload += "-\n"
                
                # LIDAR POINTS
                payload += "LOCAL_MAP\n"
                logger.debug(f"lidar points {lidar_points}")
                if lidar_points:
                    lidar_points_str = ";".join([f"{point.x:.2f} {point.y:.2f}" for point in lidar_points])
                    payload += f"{lidar_points_str}\n"
                else:
                    payload += "-\n"

                # POSITION
                payload += "POSITION\n"
                logger.debug(f"position {position}")
                if position:
                    payload += f"{position.x} {position.y} {position.th}\n"
                else:
                    payload += "-\n"

                self._socket.sendto(payload.encode(), (self._receiver_host, DATA_SERVER_CONFIG.PORT))

            except Exception as e:
                logger.error(f"DataTransmitter error: {e}")
                
            time.sleep(DATA_SERVER_CONFIG.INTERVAL)