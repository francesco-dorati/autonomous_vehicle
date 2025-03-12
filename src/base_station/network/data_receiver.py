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

    """
    def __init__(self, tcp_port: int, update_callback, buffer_size: int = 1024):
        self.tcp_port = tcp_port
        self.update_callback = update_callback  # Function to update the view with new data
        self.buffer_size = buffer_size
        self.sock = None
        self.running = False
        self.server_thread = None

    def start(self):
        """Starts the TCP server and spawns the accept/handle thread."""
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(("", self.tcp_port))
        self.sock.listen(1)  # Only one client is expected to connect.
        self.running = True
        self.server_thread = threading.Thread(target=self._accept, daemon=True)
        self.server_thread.start()
        logger.info(f"DATA RECEIVER server started on port {self.tcp_port}")

    def _accept(self):
        """
        Accepts a single connection and handles data from that client.
        When the client disconnects, it goes back to waiting for a new connection.
        """
        while self.running:
            logger.info("Waiting for client connection...")
            try:
                conn, addr = self.sock.accept()
                conn.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                logger.info(f"Accepted connection from {addr}")
                with conn:
                    while self.running:
                        data = conn.recv(self.buffer_size)
                        if not data:
                            # Client closed the connection.
                            logger.info("Client disconnected. CLOSE DATA RECEIVER")
                            break
                        ok, size, global_map, lidar_points, robot_pose = self.parse_data(data)
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

    def parse_data(self, data: bytes) -> Tuple[bool, int, np.ndarray, List[Tuple[float, float]], Tuple[float, float, float, float, float]]:
        """
        Decodifica il messaggio ricevuto e restituisce:
        - grid_np: la griglia come array NumPy di interi (con valori -1, 0, 1)
        - points: lista di tuple (int, int)
        - position: tupla di 3 interi (x, y, theta)
        """
        try:
            logger.debug(f"parsing received data, data size: {len(data)}")
            # read header
            header_format = "4sHIHHH?"
            header_size = struct.calcsize(header_format)
            header = struct.unpack(header_format, data[:header_size])
            magic, version, compressed_length, grid_size, real_grid_size, num_points, position_valid = header

            logger.debug(f"parsed header: {header}")

            if magic != b'RBT1':
                logger.error("Invalid magic number")
                return False, 0, None, None, None

            # read payload
            compressed_payload = data[header_size: header_size + compressed_length]
            payload = zlib.decompress(compressed_payload)
            checksum = zlib.crc32(payload)

            logger.debug(f"payload checksum: {checksum}")
            logger.debug(f"payload size: {len(payload)}")
            
            # --- Deserializzazione della grid ---
            # La grid è composta da grid_size * grid_size byte
            grid_data_length = real_grid_size * real_grid_size
            grid: np.ndarray = None
            if grid_data_length > 0:
                grid_bytes, payload = payload[:grid_data_length], payload[grid_data_length:]
                grid = np.frombuffer(grid_bytes, dtype=np.int8).reshape((grid_size, grid_size))

            # --- Deserializzazione dei punti ---
            points_data_length = num_points * 4  # 4 byte per punto (2 H)
            lidar_points: List[Tuple[int, int]] = None
            if points_data_length > 0:
                points_data, payload = payload[:points_data_length], payload[points_data_length:]
                points_tuple = struct.unpack(f"<{num_points * 2}h", points_data)
                lidar_points = [(points_tuple[i], points_tuple[i + 1]) for i in range(0, len(points_tuple), 2)]
            logger.debug(f"points data length: {points_data_length}")

            # --- Deserializzazione della posizione (3 int) ---
            state = None
            if position_valid:
                position_data = payload[:]
                state = struct.unpack("5d", position_data)

            return True, grid_size, grid, lidar_points, state

        except Exception as e:
            logger.error(f"PARSED DATA ERROR: {e}")
            return False, 0, None, None, None
 