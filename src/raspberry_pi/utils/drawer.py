import matplotlib.pyplot as plt
from raspberry_pi.data_structures.states import CartPoint
from raspberry_pi.config import ROBOT_CONFIG
from raspberry_pi.utils.utils import Utils
from raspberry_pi.data_structures.maps import OccupancyGrid
from raspberry_pi.utils.logger import get_logger

logger = get_logger(__name__)

import os
import datetime
import threading
from typing import List
import numpy as np
import open3d as o3d


class Drawer:
    @staticmethod
    def save_global_map(map_name, global_map: OccupancyGrid):
        """Draws the global map and save in folder."""
        def save():
            try:
                # create the folder
                logger.debug("inside draw thread")
                map_folder = ROBOT_CONFIG.MAPS_FOLDER + "/" + map_name
                logger.debug(f"Saving map on {map_folder}")
                if not os.path.exists(map_folder):
                    os.makedirs(map_folder)
                logger.debug(f"dir ok")

                # save the file
                with open(f"{map_folder}/{map_name}.txt", "w") as f:
                    logger.debug("file opened")
                    f.write(global_map.get_string(row_sep="\n"))
                logger.debug(f"Saved text on {map_folder}")
                
                # save the image
                grid = global_map.get_grid()
                if grid is not None:
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
                    logger.debug(f"Saved map on {map_folder}")
                logger.debug("End of draw thread")
            except Exception as e:
                logger.error(f"Exception in Drawer.save_global_map: {e}")

        logger.debug("Saving global map")
        t = threading.Thread(target=save, daemon=True)
        logger.debug("startng draw thread")
        t.start()
        return
        

    @staticmethod
    def draw_lidar_points(lidar_points: List[CartPoint]):
        """Draws the lidar points on the image."""
        def draw(points, timestamp):
            path = f"{ROBOT_CONFIG.SCANS_FOLDER}/lidar_points_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}.png"
            # grid_points = [(Utils.local_to_grid(size_mm, p)) for p in points]
            x = [p.x for p in points]
            y = [p.y for p in points]

            fig, ax = plt.subplots(figsize=(8, 8))  # Create figure with axes
            ax.scatter(x, y, s=1, color='blue')  # Smaller marker for a denser plot

            # Robot position (optional, adjust as necessary)
            ax.invert_yaxis()  # Invert y-axis to make the plot look correct
            ax.set_title(f"Lidar Points - {timestamp}")
            ax.set_xlabel("X")
            ax.set_ylabel("Y")
            ax.set_aspect('equal')  # Ensures aspect ratio is equal for X and Y axes
            ax.grid(True)

            # Save the plot to the path
            fig.savefig(path, format='png')
            plt.close(fig)  # Clean up the figure after saving

  
        logger.debug("\nPRINTING LIDAR POINTS\n")
        timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        t = threading.Thread(target=draw, args=(lidar_points, timestamp), daemon=True)
        t.start()
        return
    
    def draw_icp(source, target):
        # def draw(source, target):
        try:
            path = f"{ROBOT_CONFIG.SCANS_FOLDER}/icp_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}.png"
            vis = o3d.visualization.Visualizer()
            vis.create_window(visible=False)  # Create without GUI window

            # Add geometries
            vis.add_geometry(source)
            vis.add_geometry(target)

            # Render offscreen
            vis.poll_events()
            vis.update_renderer()

            # Capture and save the image
            image = vis.capture_screen_float_buffer(do_render=True)
            vis.destroy_window()

            # Convert to numpy array and save
            image = np.asarray(image)
            plt.imsave(path, image)
            logger.info("Image saved as 'icp_result.png'")

        except Exception as e:  
            logger.error(f"Exception in Drawer.draw_icp: {e}")
                
              
        
        # logger.debug("\nPRINTING ICP\n")
        # t = threading.Thread(target=draw, args=(source, target), daemon=True)
        # t.start()
        # return