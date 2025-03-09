import tkinter as tk
from typing import List, Tuple
import math
import numpy as np


from base_station.utils.logger import get_logger

logger = get_logger(__name__)

class DisplayFrame(tk.Canvas):
    def __init__(self, parent, controller):
        self.total_size = 550
        self.grid_size = 50
        self.cell_size = lambda: (self.total_size // self.grid_size) if self.grid_size > 0 else 0
        super().__init__(parent, width=self.total_size, height=self.total_size, bg="white", borderwidth=2, relief='raised')
        # super().__init__(parent)
        # real_total_size = self.cell_size() * self.grid_size
        self.image = tk.PhotoImage(width=self.total_size, height=self.total_size)
        self.image_id = self.create_image(0, 0, image=self.image, anchor="nw")
        self.tag_lower(self.image_id)
        self.controller = controller

    def show(self):
        print("SHOWING DISPLAY")
        self.pack(side='top', fill='none', expand=False, pady=10)
        # self.update(self.grid_size, None, None, None)
        
    def update(self, size: int, 
                grid: np.ndarray | None, 
                lidar_points: List[Tuple[int, int]] | None, 
                robot_pos: Tuple[int, int, int] | None):
        try:
            logger.info("UPDATING DISPLAY")
            logger.debug(f"Grid size: {size}, grid: {grid[0] if grid is not None else None} ..., lidar_points: {lidar_points[0] if lidar_points is not None else None} ..., robot_pos: {robot_pos}")

            if size != self.grid_size:
                logger.debug(f"Resizing display to {size}")
                self.grid_size = size
                # self.config(width=size * self.cell_size, height=size * self.cell_size)
                # self.image = tk.PhotoImage(width=self.total_size, height=size)  # Create new image
                self.itemconfig(self.image_id, image=self.image)  # Update the existing image on canvas

            if grid is not None:
                logger.debug("Updating grid")
                colors = {
                    -1: "#808080",  # Gray for unknown
                    0: "#FFFFFF",   # White for free space
                    1: "#000000",   # Black for obstacles
                }
                # Create a temporary image of the same size as the grid (50x50)
                temp_img = tk.PhotoImage(width=self.grid_size, height=self.grid_size)
                pixel_data = " ".join(
                    "{" + " ".join(colors[int(grid[self.grid_size - y - 1, self.grid_size - x - 1])] for x in range(self.grid_size)) + "}"
                    for y in range(self.grid_size)
                )
                temp_img.put(pixel_data)
                zoom_factor = self.total_size // self.grid_size
                self.image = temp_img.zoom(zoom_factor)
                self.itemconfig(self.image_id, image=self.image)



            # LIDAR
            self.delete("lidar")
            if lidar_points is not None:
                for x, y in lidar_points:
                    # x = -x # Invert x-axis
                    reversed_x = self.grid_size - x - 1
                    reversed_y = self.grid_size - y - 1
                    x1, y1 = reversed_x * self.cell_size(), reversed_y * self.cell_size()
                    x2, y2 = x1 + self.cell_size(), y1 + self.cell_size()
                    self.create_rectangle(y1, x1, y2, x2, fill="red", outline="", tags="lidar")

            # POS
            cx = self.total_size // 2
            cy = self.total_size // 2

            width_robot = 2 * self.cell_size()
            height_robot = 2 * self.cell_size()
            line_len = 3 * self.cell_size()
            x1 = cx - width_robot // 2
            y1 = cy - height_robot // 2
            x2 = cx + width_robot // 2
            y2 = cy + height_robot // 2
            polygon_points = [
                (x1, y1),  # Top-left
                (x2, y1),  # Top-right
                (x2, y2),  # Bottom-right
                (x1, y2)   # Bottom-left
            ]

            if robot_pos:
                rotation_rad = int(robot_pos[2])/1000
                
                arrow_x = cx - line_len*math.cos(rotation_rad)
                arrow_y = cy - line_len*math.sin(rotation_rad)
                
                rotated_polygon = []
                for (px, py) in polygon_points:
                    # Apply rotation transformation to each vertex
                    rotated_x = cx + (px - cx) * math.cos(-rotation_rad) - (py - cy) * math.sin(-rotation_rad)
                    rotated_y = cy + (px - cx) * math.sin(-rotation_rad) + (py - cy) * math.cos(-rotation_rad)
                    rotated_polygon.append(rotated_x)
                    rotated_polygon.append(rotated_y)
                # x1 = cx + (x1 - cx) * math.cos(rotation_rad) - (y1 - cy) * math.sin(rotation_rad)
                # y1 = cy + (x1 - cx) * math.sin(rotation_rad) - (y1 - cy) * math.cos(rotation_rad)
                # x2 = cx + (x2 - cx) * math.cos(rotation_rad) + (y2 - cy) * math.sin(rotation_rad)
                # y2 = cy + (x2 - cx) * math.sin(rotation_rad) + (y2 - cy) * math.cos(rotation_rad)
            else:
                arrow_x = cx - line_len
                arrow_y = cy 
                rotated_polygon = [point for p in polygon_points for point in p]

            self.delete("robot_rect") 
            self.create_polygon(rotated_polygon, fill='blue', width=2, tags="robot_rect")
            self.delete("pos_x")
            self.create_line(cy, cx, arrow_y, arrow_x, fill="red", width=2, arrow=tk.LAST, arrowshape=(3, 4, 4), tags="pos_x")


        except Exception as e:
            logger.error(f"UPDATE DISPLAY ERROR: {e}")
            return
        
        
        #pack(side='top', fill='both', expand=True, pady=10)
