import tkinter as tk
from typing import List, Tuple
import math

class DisplayFrame(tk.Canvas):
    def __init__(self, parent, controller):
        super().__init__(parent)
        total_size = 550
        self.grid_size = 50
        self.cell_size = lambda: total_size // self.grid_size
        real_total_size = self.cell_size() * self.grid_size
        self.config(borderwidth=2, relief='raised', bg='lightgray', width=real_total_size, height=real_total_size)
        
        self.controller = controller

    def show(self):
        print("SHOWING DISPLAY")
        self.pack(side='top', fill='none', expand=False, pady=10)
        # self.update(self.grid_size, None, None, None)
        
    def update(self, size: int, 
                grid: List[List[int]], 
                lidar_points: List[Tuple[int, int]], 
                robot_pos: Tuple[int, int, int]):
        print("UPDATING DISPLAY")
        self.delete("all")
        self.grid_size = size
        if grid:
            for row in range(self.grid_size):
                for col in range(self.grid_size):
                    x1, y1 = col * self.cell_size(), row * self.cell_size()
                    x2, y2 = x1 + self.cell_size(), y1 + self.cell_size()
                    color = "black"
                    if grid:
                        v = grid[row][col]
                        if v == -1:
                            color = "gray"
                        elif v == 0:
                            color = "white"
                        elif v == 1:
                            color = "black"
                    self.create_rectangle(y1, x1, y2, x2, fill=color, outline="")

        if lidar_points:
            for x, y in lidar_points:
                # x = -x # Invert x-axis
                reversed_x = self.grid_size - x - 1
                reversed_y = self.grid_size - y - 1
                x1, y1 = reversed_x * self.cell_size(), reversed_y * self.cell_size()
                x2, y2 = x1 + self.cell_size(), y1 + self.cell_size()
                self.create_rectangle(y1, x1, y2, x2, fill="red", outline="")

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
        self.create_line(cy, cx, fy1, fx1, fill="red", width=2, arrow=tk.LAST, arrowshape=(3, 4, 4))
        self.create_line(cy, cx, fy2, fx2, fill="green", width=2, arrow=tk.LAST, arrowshape=(3, 4, 4))

        
        #pack(side='top', fill='both', expand=True, pady=10)
