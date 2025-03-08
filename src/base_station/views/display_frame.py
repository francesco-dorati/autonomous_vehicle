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

            self.delete("robot_rect") 
            width_pixels = 2 * self.cell_size()
            height_pixels = 2 * self.cell_size()
            x1 = cx - width_pixels // 2
            y1 = cy - height_pixels // 2
            x2 = cx + width_pixels // 2
            y2 = cy + height_pixels // 2
            self.create_rectangle(x1, y1, x2, y2, fill='blue', width=2, tags="robot_rect")

            self.delete("pos_x")
            self.delete("pos_y")
            line_len = 3*self.cell_size()
            if robot_pos:
                fx1 = cx - line_len*math.cos(int(robot_pos[2])/1000)
                fy1 = cy - line_len*math.sin(int(robot_pos[2])/1000)
                fx2 = cx + line_len*math.sin(int(robot_pos[2])/1000)
                fy2 = cy - line_len*math.cos(int(robot_pos[2])/1000)
            else:
                fx1 = cx - line_len
                fy1 = cy
                fx2 = cx 
                fy2 = cy - line_len
            self.create_line(cy, cx, fy1, fx1, fill="red", width=2, arrow=tk.LAST, arrowshape=(3, 4, 4), tags="pos_x")
            self.create_line(cy, cx, fy2, fx2, fill="green", width=2, arrow=tk.LAST, arrowshape=(3, 4, 4), tags="pos_y")

        except Exception as e:
            logger.error(f"UPDATE DISPLAY ERROR: {e}")
            return
        
        
        #pack(side='top', fill='both', expand=True, pady=10)
