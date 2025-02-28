import socket
import threading
import time
from typing import Tuple, List
import numpy as np
import struct
import zlib

from base_station.utils.logger import get_logger

logger = get_logger(__name__)

class DataReceiver:
    """
    A simple TCP server that listens on a specific port (e.g., 5502)
    for display data and uses an update callback to notify the view.

    Expected data format from the client:
        DATA
        <map_pixel_size>
        GLOBAL_MAP
        <global map string (or '-' if empty)>
        LOCAL_MAP
        <lidar points string (or '-' if empty)>
        POSITION
        <x> <y> <theta>  (or '-' if not available)
    """
    def __init__(self, tcp_port: int, update_callback, buffer_size: int = 4086):
        self.tcp_port = tcp_port
        self.update_callback = update_callback  # Function to update the view with new data
        self.buffer_size = buffer_size
        self.sock = None
        self.running = False
        self.server_thread = None

    def start(self):
        """Starts the TCP server and spawns the accept/handle thread."""
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.bind(("", self.tcp_port))
        self.sock.listen(1)  # Only one client is expected to connect.
        self.running = True
        self.server_thread = threading.Thread(target=self._accept_and_handle, daemon=True)
        self.server_thread.start()
        logger.info(f"DATA RECEIVER server started on port {self.tcp_port}")

    def _accept_and_handle(self):
        """
        Accepts a single connection and handles data from that client.
        When the client disconnects, it goes back to waiting for a new connection.
        """
        while self.running:
            logger.info("Waiting for client connection...")
            try:
                conn, addr = self.sock.accept()
                logger.info(f"Accepted connection from {addr}")
                with conn:
                    while self.running:
                        data = conn.recv(self.buffer_size)
                        if not data:
                            # Client closed the connection.
                            logger.info("Client disconnected. CLOSE DATA RECEIVER")
                            break
                        data_str = data.decode().strip()
                        ok, size, global_map, lidar_points, robot_pose = self.parse_data(data_str)
                        if ok:
                            self.update_callback(size, global_map, lidar_points, robot_pose)
            except Exception as e:
                logger.error("Error in connection handling:", e)
                # If an error occurs, break out of the loop and stop the server.
                break

        logger.info("Exiting server accept/handle loop.")
        self.running = False
        if self.sock:
            self.sock.close()

    def stop(self):
        """Stops the TCP server and cleans up resources."""
        self.running = False
        if self.sock:
            self.sock.close()
        if self.server_thread and threading.current_thread() != self.server_thread:
            self.server_thread.join()
        logger.info("TCP server stopped.")

    def parse_data(self, data: bytes) -> Tuple[bool, int, np.ndarray, List[Tuple[float, float]], Tuple[float, float, float]]:
        """
        Decodifica il messaggio ricevuto e restituisce:
        - grid_np: la griglia come array NumPy di interi (con valori -1, 0, 1)
        - points: lista di tuple (int, int)
        - position: tupla di 3 interi (x, y, theta)
        """
        try:
            logger.debug("parsing received data, data size: ", len(data))
            # read header
            header_format = "4sHIHH?"
            header_size = struct.calcsize(header_format)
            header = struct.unpack(header_format, data[:header_size])
            magic, version, compressed_length, grid_size, num_points, position_valid = header

            logger.debug(f"parsed header: {header}")

            if magic != b'RBT1':
                logger.error("Invalid magic number")
                return False, 0, None, None, None

            # read payload
            compressed_payload = data[header_size: header_size + compressed_length]
            payload = zlib.decompress(compressed_payload)

            # --- Deserializzazione della grid ---
            # La grid è composta da grid_size * grid_size byte
            grid_np = None
            if grid_size != 0:
                grid_data_length = grid_size * grid_size
                grid_bytes = payload[:grid_data_length]
                grid_np: np.ndarray = np.frombuffer(grid_bytes, dtype=np.int8).reshape((grid_size, grid_size))

            # --- Deserializzazione dei punti ---
            lidar_points = None
            if num_points != 0:
                points_data_start = grid_data_length
                points_data_end = points_data_start + num_points * 4  # 4 byte per punto (2 H)
                points_data = payload[points_data_start: points_data_end]
                points_tuple = struct.unpack(f"{num_points * 2}i", points_data)
                lidar_points: List[Tuple[int, int]] = [(points_tuple[i], points_tuple[i + 1]) for i in range(0, len(points_tuple), 2)]

            # --- Deserializzazione della posizione (3 int) ---
            position = None
            if position_valid:
                position_data = payload[points_data_end:]
                position: Tuple[int, int, int] = struct.unpack("3i", position_data)

            return True, grid_size, grid_np, lidar_points, position

        except Exception as e:
            logger.error("PARSED DATA ERROR: exception", e)
            return False, 0, None, None, None
 