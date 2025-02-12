import threading
import matplotlib.pyplot as plt
from raspberry_pi.data_structures.states import CartPoint
from raspberry_pi.config import ROBOT_CONFIG
from raspberry_pi.utils.utils import Utils

from typing import List
import datetime


class Drawer:
    @staticmethod
    def draw_global_map(map_name, global_map):
        """Draws the global map and save in folder."""
        pass

    @staticmethod
    def draw_lidar_points(size_mm, lidar_points: List[CartPoint]):
        """Draws the lidar points on the image."""
        def draw(points, timestamp):
            path = f"{ROBOT_CONFIG.SCANS_FOLDER}/lidar_points_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}.png"
            grid_points = [Utils.local_to_grid(size_mm, p) for p in points]
            x = [p[0] for p in grid_points]
            y = [p[1] for p in grid_points]
            grid_size = Utils.dist_to_grid(size_mm)
            plt.scatter(x, y, color='red', marker='o', s=100, label="Grid Points")
            plt.grid(True, linestyle="--", linewidth=0.5)
            plt.xticks(range(0, grid_size))  # Set integer ticks on x-axis
            plt.yticks(range(0, grid_size))  # Set integer ticks on y-axis
            plt.xlabel("X-axis")
            plt.ylabel("Y-axis")
            plt.title(f"Lidar Points - {timestamp}")
            plt.savefig(path, dpi=300)  # Save with high resolution
            plt.close()
        timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        t = threading.Thread(target=draw, args=(lidar_points, timestamp))
        t.start()
        return
       