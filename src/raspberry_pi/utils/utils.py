from raspberry_pi.config import ROBOT_CONFIG
#from raspberry_pi.data_structures.states import CartPoint

import numpy as np
from typing import Tuple, List
class Utils:
    PI_RAD = 3.14159265359
    PI_MRAD = 3141.592653
    MRAD_DEG = 17.4532925199
    @staticmethod
    def normalize_rad(rad: float) -> float:
        return (rad + Utils.PI_RAD) % (2*Utils.PI_RAD) - Utils.PI_RAD
    
    @staticmethod
    def normalize_mrad(mrad: int) -> int:
        return (mrad + Utils.PI_MRAD) % (2*Utils.PI_MRAD) - Utils.PI_MRAD

    @staticmethod
    def mrad_to_deg(mrad: int) -> float:
        return mrad / Utils.MRAD_DEG
    
    @staticmethod
    def deg_to_mrad(deg: float) -> int:
        return int(deg * Utils.MRAD_DEG)
    
    @staticmethod
    def dist_to_grid(dist_m: float) -> int:
        return dist_m // ROBOT_CONFIG.GLOBAL_MAP_RESOLUTION
    
    @staticmethod
    def local_to_grid(local_points: np.array, grid_size_m: int) -> np.array:
        """Converts a local point to grid point."""
        grid_points = (local_points / ROBOT_CONFIG.GLOBAL_MAP_RESOLUTION) + (Utils.dist_to_grid(grid_size_m) / 2)
        return np.round(grid_points).astype(int)
    
        grid_points = []
        grid_size = Utils.dist_to_grid(grid_size_mm)
        for point in local_point:
            gx = int(round(point.x / ROBOT_CONFIG.GLOBAL_MAP_RESOLUTION) + (grid_size // 2))
            gy = int(round(point.y / ROBOT_CONFIG.GLOBAL_MAP_RESOLUTION) + (grid_size // 2))
            grid_points.append((gx, gy))
        return grid_points
    
    # @staticmethod
    # def local_to_world(local_point: CartPoint, position: Position) -> CartPoint:
    #     """Converts a local point to global point."""
    #     x = local_point.x * Utils.mrad_to_deg(position.th) + position.x
    #     y = local_point.y * Utils.mrad_to_deg(position.th) + position.y
    #     return CartPoint(x, y)
    
    