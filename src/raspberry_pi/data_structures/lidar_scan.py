"""
    Lidar Scan
    [lock]
    
    ### PUBLIC ###
    LidarScan()
        initializes the lidar scan

    add_sample(angle_deg: float, dist_mm: float) -> None
        [lock]
        adds sample to the scan
        args:
            angle_deg (float): angle of the sample
            dist_mm (int): distance of the obstacle
    
            
    get_copy() -> list[tuple[int, int]]
        [lock]
        generates a copy of the scan
        returns:
            list[tuple[int, int]]
    

    ### PRIVATE ###
    _scan: np.array
        array of the scan
        index: angle
        value: distance

    _scan_lock: threading.Lock
        lock for the scan
"""

import threading
import numpy as np

class LidarScan:
        def __init__(self):
            """
            Initializes the Lidar Scan
            """
            self._scan = np.full(360, 0, dtype=tuple)
            self._scan_lock = threading.Lock()

        def add_sample(self, angle_deg: float, dist_mm: float) -> None:
            """ LOCK
            Adds sample to the scan
            Args:
                angle_deg (float): angle of the sample, 
                dist_mm (int): distance of the obstacle
            """
            angle_deg = round(angle_deg) % 360
            with self._scan_lock:
                self._scan[angle_deg] = int(dist_mm)
                # print("add sample ", angle_deg, dist_mm)

        def get_copy(self) -> list[tuple[int, int]]:
            """ LOCK
            Generates a copy of the scan
            Returns:
                list[tuple[int, int]]
            """
            with self._scan_lock:
                copy = self._scan.copy()
            print("copy of ", copy)
            return copy