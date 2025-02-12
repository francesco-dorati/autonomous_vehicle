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
            print("\nREALLY PRINTING LIDAR POINTS\n")
            path = f"{ROBOT_CONFIG.SCANS_FOLDER}/lidar_points_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}.png"
            grid_points = [(Utils.local_to_grid(size_mm, p)) for p in points]
            x = [p[0] for p in grid_points]
            y = [p[1] for p in grid_points]
            grid_size = Utils.dist_to_grid(size_mm)

            plt.figure(figsize=(8, 8))
            plt.scatter(x, y, s=1, color='blue')  # Smaller marker for a denser plot
            # Robot position
            plt.gca().invert_yaxis()  # Invert y-axis to make the plot look correct
            plt.title("LIDAR Scan")
            plt.xlabel("X")
            plt.ylabel("Y")
            plt.title(f"Lidar Points - {timestamp}")
            plt.axis('equal')  # Ensures aspect ratio is equal for X and Y axes
            plt.grid(True)
            plt.savefig(path, format='png')
            plt.close() 
            ######
  
        print("\nPRINTING LIDAR POINTS\n")
        timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        t = threading.Thread(target=draw, args=(lidar_points, timestamp), daemon=True)
        t.start()
        return
       