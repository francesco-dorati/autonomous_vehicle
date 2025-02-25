import matplotlib.pyplot as plt
from raspberry_pi.data_structures.states import CartPoint
from raspberry_pi.config import ROBOT_CONFIG
from raspberry_pi.utils.utils import Utils
from raspberry_pi.data_structures.maps import OccupancyGrid

import os
import datetime
import threading
from typing import List


class Drawer:
    @staticmethod
    def save_global_map(map_name, global_map: OccupancyGrid):
        """Draws the global map and save in folder."""
        def save():
            # create the folder
            map_folder = ROBOT_CONFIG.MAPS_FOLDER + "/" + map_name
            if not os.path.exists(map_folder):
                os.makedirs(map_folder)

            # save the file
            with open(f"{map_folder}/{map_name}.txt", "w") as f:
                f.write(global_map.get_map_string(row_sep="\n"))
            
            # save the image
            plt.figure(figsize=(8, 8))
            plt.imshow(global_map.get_grid(), cmap='gray')
            plt.gca().invert_yaxis()
            plt.title("Global Map")
            plt.xlabel("X")
            plt.ylabel("Y")
            plt.title(f"Global Map - {map_name}")
            plt.axis('equal')
            plt.grid(True)
            plt.savefig(f"{map_folder}/{map_name}.png", format='png')
            plt.close()
    
        t = threading.Thread(target=save, daemon=True)
        t.start()
        return
        

    @staticmethod
    def draw_lidar_points(size_mm, lidar_points: List[CartPoint]):
        """Draws the lidar points on the image."""
        def draw(points, timestamp):
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

  
        print("\nPRINTING LIDAR POINTS\n")
        timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        t = threading.Thread(target=draw, args=(lidar_points, timestamp), daemon=True)
        t.start()
        return
       