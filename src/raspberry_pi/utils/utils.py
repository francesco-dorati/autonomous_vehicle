from raspberry_pi.data_structures.states import CartPoint
from raspberry_pi.config import ROBOT_CONFIG

from typing import Tuple
class Utils:
    PI_MRAD = 3141.592653
    MRAD_DEG = 17.4532925199
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
    def dist_to_grid(self, dist_mm: float) -> int:
        return dist_mm // ROBOT_CONFIG.GLOBAL_MAP_RESOLUTION
    
    @staticmethod
    def local_to_grid(grid_size_mm: int, local_point: CartPoint) -> Tuple[int, int]:
        """Converts a local point to grid point."""
        grid_size = Utils.dist_to_grid(grid_size_mm)
        gx = int(round(local_point.x / ROBOT_CONFIG.GLOBAL_MAP_RESOLUTION) + (grid_size // 2))
        gy = int(round(local_point.y / ROBOT_CONFIG.GLOBAL_MAP_RESOLUTION) + (grid_size // 2))
        return gx, gy
    
    