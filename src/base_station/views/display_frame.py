import tkinter as tk
from typing import List, Tuple
import math
import numpy as np
from base_station.utils.logger import get_logger

logger = get_logger(__name__)

class DisplayFrame(tk.Canvas):
    def __init__(self, parent, controller):
        total_size = 550
        self.grid_size = 50
        self.cell_size = lambda: (total_size // self.grid_size) if self.grid_size > 0 else 0
        super().__init__(parent, width=total_size, height=total_size, bg="white")
        # super().__init__(parent)
        real_total_size = self.cell_size() * self.grid_size
        self.image = tk.PhotoImage(width=self.grid_size, height=self.grid_size)
        self.image_id = self.create_image((0, 0), image=self.image, anchor="nw")
        # self.config(borderwidth=2, relief='raised', bg='lightgray', width=real_total_size, height=real_total_size)
        
        self.controller = controller

    def show(self):
        print("SHOWING DISPLAY")
        self.pack(side='top', fill='none', expand=False, pady=10)
        # self.update(self.grid_size, None, None, None)
        
    def update(self, size: int, 
                grid: np.ndarray | None, 
                lidar_points: List[Tuple[int, int]] | None, 
                robot_pos: Tuple[int, int, int] | None):
        logger.info("UPDATING DISPLAY")
        logger.debug(f"Grid size: {size}, grid: {grid}, lidar_points: {lidar_points}, robot_pos: {robot_pos}")

        if size != self.grid_size:
            self.grid_size = size
            # self.config(width=size * self.cell_size, height=size * self.cell_size)
            self.image = tk.PhotoImage(width=size, height=size)  # Create new image
            self.itemconfig(self.image_id, image=self.image)  # Update the existing image on canvas

        if grid is not None:
            colors = {
                -1: "#808080",  # Gray for unknown
                0: "#FFFFFF",   # White for free space
                1: "#000000",   # Black for obstacles
            }
            pixels = "\n".join(" ".join(colors[grid[x, y]] for x in range(self.size)) for y in range(self.size))
            self.image.put(pixels)

        self.delete("lidar")
        self.delete("pos_x")
        self.delete("pos_y")
        
        # self.delete("all")
        # self.grid_size = size
        # if grid is not None:
        #     for x in range(self.grid_size):
        #         for y in range(self.grid_size):
        #             # reversed_x = self.grid_size - x - 1
        #             reversed_y = self.grid_size - y - 1
        #             x1, y1 = x * self.cell_size(), reversed_y * self.cell_size()
        #             x2, y2 = x1 + self.cell_size(), y1 + self.cell_size()
        #             v = grid[x][reversed_y]
        #             if v == -1:
        #                 color = "gray"
        #             elif v == 0:
        #                 color = "white"
        #             elif v == 1:
        #                 color = "black"
        #             self.create_rectangle(y1, x1, y2, x2, fill=color, outline="")

        if lidar_points:
            for x, y in lidar_points:
                # x = -x # Invert x-axis
                reversed_x = self.grid_size - x - 1
                reversed_y = self.grid_size - y - 1
                x1, y1 = reversed_x * self.cell_size(), reversed_y * self.cell_size()
                x2, y2 = x1 + self.cell_size(), y1 + self.cell_size()
                self.create_rectangle(y1, x1, y2, x2, fill="red", outline="", tags="lidar")

        # POS
        cx = self.cell_size()*self.grid_size // 2
        cy = self.cell_size()*self.grid_size // 2
        line_len = 3*self.cell_size()
        if robot_pos:
            fx1 = cx - line_len*math.cos(int(robot_pos[2])/1000)
            fy1 = cy - line_len*math.sin(int(robot_pos[2])/1000)
            fx2 = cx - line_len*math.sin(int(robot_pos[2])/1000)
            fy2 = cy - line_len*math.cos(int(robot_pos[2])/1000)
        else:
            fx1 = cx - line_len
            fy1 = cy
            fx2 = cx 
            fy2 = cy - line_len
        self.create_line(cy, cx, fy1, fx1, fill="red", width=2, arrow=tk.LAST, arrowshape=(3, 4, 4), tags="pos_x")
        self.create_line(cy, cx, fy2, fx2, fill="green", width=2, arrow=tk.LAST, arrowshape=(3, 4, 4), tags="pos_y")

        
        #pack(side='top', fill='both', expand=True, pady=10)
