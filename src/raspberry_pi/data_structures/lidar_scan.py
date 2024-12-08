import threading
import numpy as np

class LidarScan:
        def __init__(self):
            self._last_scan_id = 0
            self._scan = np.full(360, 0, dtype=tuple)
            self._scan_lock = threading.Lock()

        def add_sample(self, angle_deg: float, dist_mm: int) -> None:
            """
            Add Sample
            Adds sample to the scan

            Args:
                angle_deg (float): angle of the sample, 
                dist_mm (int): distance of the obstacle
            """
            with self._scan_lock:
                angle_deg = round(angle_deg) % 360
                self._scan[angle_deg] = int(dist_mm)

        def get_copy(self) -> list:
            """
            Gat Local Map
            Generates local map based on the scan

            Returns:
                LocalMap: local map based on the scan
            """
            with self._scan_lock:
                copy = self._scan.copy()
            return copy