"""
    LocalMap
    ### PUBLIC ###
    methods:
        LocalMap(scan: LidarScan)
            initializes the local map
            args:
              scan (LidarScan): LidarScan object

    # TODO finish the docstring

GlobalMap

Returns:
    _type_: _description_
"""

import math
import os
import numpy as np
from typing import Optional, List, Tuple
import threading
import copy

from raspberry_pi.data_structures.states import Position, PolarPoint, CartPoint
# from raspberry_pi.utils import Utils
from raspberry_pi.config import ROBOT_CONFIG
from raspberry_pi.utils.logger import get_logger, timing_decorator
from raspberry_pi.utils.utils import Utils

logger = get_logger(__name__)

class LidarScan:
        def __init__(self):
            """
            Initializes the Lidar Scan
            """
            self.__scan = np.full(360, 0, dtype=tuple)
            self.__scan_lock = threading.Lock()

        def add_sample(self, angle_deg: float, dist_mm: float) -> None:
            """ LOCK
            Adds sample to the scan
            Args:
                angle_deg (float): angle of the sample, 
                dist_mm (int): distance of the obstacle
            """
            angle_deg = round(angle_deg) % 360
            with self.__scan_lock:
                self.__scan[angle_deg] = int(dist_mm)
                # print("add sample ", angle_deg, dist_mm)

        def get_copy(self) -> list[PolarPoint]:
            """ LOCK
            Generates a copy of the scan
            Returns:
                list[tuple[int, int]]
            """
            copy = []
            with self.__scan_lock:
                for ang_deg, dist in enumerate(self.__scan):
                    ang_mrad = Utils.deg_to_mrad(ang_deg)
                    copy.append(PolarPoint(dist, ang_mrad))
            return copy

class OccupancyGrid:
    def __init__(self, size_mm: int, resolution_mm: int):
        self.__size_mm: int = size_mm
        self.__resolution_mm: int = resolution_mm
        self.__grid_size: int = size_mm // resolution_mm
        self.__grid = None
    
    def get_size_mm(self):
        return self.__size_mm

    def get_grid_size(self):
        return self.__grid_size
    
    
    def set(self, gx, gy, val):
        if self.__grid is None:
            self.__grid = np.full((self.__grid_size, self.__grid_size), -1, dtype=int)

        self.__grid[gx][gy] = val

    def origin(self):
        return ((self.__grid_size // 2), (self.__grid_size // 2))

    def local_to_grid(self, p: CartPoint) -> Tuple[int, int]:
        gx = int(round(p.x / self.__resolution_mm) + (self.__grid_size // 2))
        gy = int(round(p.y / self.__resolution_mm) + (self.__grid_size // 2))
        return gx, gy
    
    def get_string(self):
        if self.__grid is None:
            return "-"
        return ";".join([" ".join(map(str, row)) for row in self.__grid])


# can be extended as localmap as abstarct class and lidarmap, occupancy map as childs
class LocalMap:
    def __init__(self, scan: List):
        """
        Local Map
        A list of position taken at 1° steps
        __scan (List): list of (angle, distance) tuples
        Arguments:
            Lidar.Scan: scan object 
        """
        # print(scan)
        self.__scan: List[PolarPoint] = scan
        # print("SCAN: ", self._scan)

    def __repr__(self):
        return f"local_map()"

 
    @timing_decorator
    def get_polar_points(self, section_radius: int = None) -> List[PolarPoint]:
        """Get polar coordinates of the scan
        Returns:
            List[float, int]: list of (angle, dist)
        """
        if section_radius is None:
            return [coord for coord in self.__scan]
        return [coord for coord in self.__scan if coord.distance <= section_radius]

    @timing_decorator
    def get_cartesian_points(self, section_size: int = None) -> List[CartPoint]:
        """Get cartesian coordinates of the scan (based on the center of the local map)
        Returns:
            List[CartPoint]: list cartesian points in local coordinates (refers to position of the robot)
        """
        points = [] 
        # print(self.__scan)
        for p in self.__scan:
            # Convert angle from degrees to radians for trigonometric functions
            # logger.debug(f"p: {p}")
            angle_rad: float = p.th / 1000.0        # p.th already normalized
            if p.r > 0:
                cart_p = CartPoint(
                    p.r * np.cos(angle_rad), 
                    p.r * np.sin(angle_rad)
                )
                max_dist = (section_size//2)*ROBOT_CONFIG.GLOBAL_MAP_RESOLUTION
                inside_section = section_size is None or (abs(cart_p.x) < max_dist and abs(cart_p.y) < max_dist)
                if inside_section:
                    points.append(cart_p)
        logger.debug(points)
        return points

    @timing_decorator
    def get_occupancy_grid(self) -> OccupancyGrid:
        """Get the occupancy grid of the local map
        Returns:
            OccupancyGrid: 
        """
        pass

    @timing_decorator
    def is_position_free(self, pos: Position) -> bool:
        """
        Is Position Free
        Returns if position is obstacle free
        Position is relative to the local map
        Args:
            pos (Position): relative position to the center of the local map
        Returns:
            bool: if position given is obstacle free
        """
        pass

    @timing_decorator
    def is_line_free(self, pos1: Position, pos2: Position) -> bool:
        """
        Is Line Free
        Returns if the segment connecting the two points is free
        Positions are relative to the local map
        Args:
            pos1 (Position): starting point of the segment
            pos2 (Position): ending point of the segment
        Returns:
            bool: if segment is obstacle free
        """
        pass

    @timing_decorator
    def is_area_free(self, angle1, angle2, dist_mm) -> bool:
        """
        Is Area Free
        Returns if the area between the two angles and the distance is free
        Args:
            angle1 (_type_): _description_
            angle2 (_type_): _description_
            dist_mm (_type_): _description_
        Returns:
            bool: if the area is free
        """
        pass

    
    # @timing_decorator
    # def draw(self, filename) -> None:
    #     import matplotlib.pyplot as plt
    #     path_name = f"../img/{filename}.png"
    #     x_points = []
    #     y_points = []
    #     for angle, distance in enumerate(self._scan):
    #         if distance == 0: 
    #             continue
    #         # Convert angle from degrees to radians for trigonometric functions
    #         angle_rad = np.radians(360 - angle)
    #         # Skip points with invalid or zero distance
    #         if distance > 0:
    #             x = distance * np.cos(angle_rad)
    #             y = distance * np.sin(angle_rad)
    #             x_points.append(x)
    #             y_points.append(y)
    #     # Plotting
    #     plt.figure(figsize=(8, 8))
    #     plt.scatter(x_points, y_points, s=1, color='blue')  # Smaller marker for a denser plot
    #     # Robot position
    #     plt.plot(0, 0, 'ro', label="Robot Position")  # Red dot for robot location
    #     plt.arrow(0, 0, 500, 0, head_width=100, head_length=80, fc='red', ec='red', label="Heading")
    #     plt.gca().invert_yaxis()  # Invert y-axis to make the plot look correct
    #     plt.title("LIDAR Scan")
    #     plt.xlabel("X (mm)")
    #     plt.ylabel("Y (mm)")
    #     plt.axis('equal')  # Ensures aspect ratio is equal for X and Y axes
    #     plt.grid(True)
    #     plt.savefig(path_name, format='png')
    #     plt.close() 

    
class GlobalMap:
    """
    Global Map
    Fixed resolution (100mm)
    Variable dimentions
    Occupancy grid
        -1 unknown
        0 free
        1 occupied
    (can be extended to a probability map returning what is the chance of a point to be occupied)
    """

    def __init__(self, name: str):
        self.name: str = name
        self._grid_size: int = ROBOT_CONFIG.GLOBAL_MAP_SIZE_MM // ROBOT_CONFIG.GLOBAL_MAP_RESOLUTION

        self._grid = np.full((self._grid_size, self._grid_size), -1, dtype=int)
        self.origin = lambda: ((self._grid_size // 2), (self._grid_size // 2))
    
    def __repr__(self):
        return f"global_map(size={self._grid_size})"

    @timing_decorator
    def is_position_free(self, pos: Position) -> bool:
        """
        Is Position Free
        Returns if position is obstacle free

        Args:
            pos (Position): global position

        Returns:
            bool: if position is free
        """
        gx, gy = self._world_to_grid(pos)
        return self._grid[gx][gy] == 0

    @timing_decorator
    def is_segment_free(self, pos1: Position, pos2: Position) -> bool:
        """
        Is Segment Free
        Returns if the segment connecting the two points is free

        Args:
            pos1 (Position): starting point of the segment
            pos2 (Position): ending point of the segment

        Returns:
            bool: if segment is obstacle free
        """
        pass

    @timing_decorator
    def expand(self, position: Position, local_map: LocalMap) -> None:
        """
        Expand
        Expands the map based on the local map and its position

        Args:
            position (Position): position of the local map
            local_map (LocalMap): local map of the envoironment
        """
        local_points = local_map.get_certeisan_points()
        for lx, ly in local_points:
            robot_th_rad = position.th / 1000
            obs_x = position.x + lx*np.cos(robot_th_rad) - ly*np.sin(robot_th_rad)
            obs_y = position.y + lx*np.sin(robot_th_rad) + ly*np.cos(robot_th_rad)
            obs_point = CartPoint(obs_x, obs_y) # global coordinates
            robot_point = position.get_point()
            if not self._is_inside(obs_point):
                self._expand_grid()
            self._ray_cast(robot_point, obs_point)

    def get_subsection(self, center_local: CartPoint, size: int) -> OccupancyGrid:
        """
        Get Subsection
        Returns a subsection of the grid based on the position and size

        Args:
            position (Position): center of the subsection
            size (int): size of the subsection

        Returns:
            OccupancyGrid: subsection of the grid
        """
        subsection = OccupancyGrid(size)
        for lx in range(size):
            for ly in range(size):
                current_global = CartPoint(
                    center_local.x + lx,
                    center_local.y + ly
                )
                val = -1
                if self._is_inside(current_global):
                    val = self._get(current_global)
                subsection.set(lx, ly, val)
        return subsection



    # @timing_decorator
    # @staticmethod
    # def save(self, name: str, grid: np.ndarray) -> None:
    #     import matplotlib.pyplot as plt
    #     from matplotlib.colors import ListedColormap

    #     path_name = f"../data/maps/{name}"
    #     os.makedirs(path_name, exist_ok=True)
    #     image_name = f"{path_name}/{name}.png"
    #     text_name = f"{path_name}/{name}.txt"
    #     # TEXT
    #     with open(text_name, 'w') as file:
    #         file.write(f"{self._grid_size}\n")
    #         for row in self._grid:
    #             file.write(" ".join(map(str, row)) + "\n")
        
    #     # IMAGE
    #     cmap = ListedColormap(['gray', 'white', 'black'])
    #     bounds = [-1.5, -0.5, 0.5, 1.5]  # Boundaries for each value
    #     norm = plt.matplotlib.colors.BoundaryNorm(bounds, cmap.N)
    #     plt.figure(figsize=(8, 8))
    #     plt.imshow(grid.T, cmap=cmap, norm=norm, origin='upper')
    #     # Robot position
    #     y, x = self.origin()
    #     plt.plot(y, x, 'ro', label="Robot Position")  # Red dot for robot location
    #     plt.arrow(y, x, 3, 0, head_width=1, head_length=1, fc='red', ec='red', label="Heading")
    #     plt.arrow(y, x, 0, 3, head_width=1, head_length=1, fc='green', ec='green', label="Heading")
    #     plt.gca().invert_yaxis()  # Invert y-axis to make the plot look correct
    #     plt.title("LIDAR Scan")
    #     plt.xlabel("X (mm)")
    #     plt.ylabel("Y (mm)")
    #     plt.axis('equal')  # Ensures aspect ratio is equal for X and Y axes
    #     plt.grid(True)
    #     plt.savefig(path_name, format='png')
    #     plt.close() 

    # get grid:  position, size -> array (to send to dev)
    # get grid:  position, size -> 01110110101



    def _expand_grid(self) -> None:
        new_size = (self._grid_size*2)
        new_grid = np.full((new_size, new_size), -1, dtype=int)
        off = (new_size - self._grid_size) // 2

        new_grid[off:off+self._grid_size, off:off+self._grid_size] = self._grid
        self._grid_size = new_size
        self._grid = new_grid

    def _world_to_grid(self, p: CartPoint) -> tuple[int, int]:
        """
        World Position to Grid Position
        Position (0, 0) world is center of grid

        Args:
            pos (Position): position reference to the starting point

        Returns:
            int, int: index of the position
        """
        gx = round(p.x / self.RESOLUTION) + (self._grid_size // 2)
        gy = round(p.y / self.RESOLUTION) + (self._grid_size // 2)
        return gx, gy
    
    def _is_inside(self, p: CartPoint):
        gx, gy = self._world_to_grid(p)
        inside_x = (gx >= 0 and gx < self._grid_size)
        inside_y = (gy >= 0 and gy < self._grid_size)
        inside = inside_x and inside_y
        return inside
        
    def _get(self, p: CartPoint) -> int:
        gx, gy = self._world_to_grid(p)
        return self._grid[gx][gy]
    
    def _set_unknown(self, gx, gy) -> None:
        self._grid[gx][gy] = -1
    
    def _set_free(self, gx, gy) -> None:
        self._grid[gx][gy] = 0
    
    def _set_occupied(self, gx, gy) -> None:
        self._grid[gx][gy] = 1 

    def _ray_cast(self, robot_point: CartPoint, obs_point: CartPoint) -> None:
        """
        Ray Cast
        Set the line between robot and obstacle as free
            and the obstacle position as occupied

        Args:
            robot_pos (Position): position of the robot
            obstacle_pos (Position): position of the obstacle
        """
        xr, yr = self._world_to_grid(robot_point)
        xo, yo = self._world_to_grid(obs_point)
        cells_between = self._cells_between(xr, yr, xo, yo)
        for gx, gy in cells_between:
            self._set_free(gx, gy)
        self._set_occupied(xo, yo)
    
    def _cells_between(self, x0, y0, x1, y1) -> list:
        """
        Cells Between
        Returns:
            list: list of (gx, gy), including initial and final values [(gx0, gy0), ..., (gx1, gy1)]
        """
        if abs(x1-x0) > abs(y1-y0):
            # HORIZONTAL
            cells = self._cells_between_h(x0, y0, x1, y1)
        else:
            # VERTICAL
            cells = self._cells_between_v(x0, y0, x1, y1)
        return cells
    
    def _cells_between_h(self, x0, y0, x1, y1):
        cells = []
        if x0 > x1:
            x0, x1 = x1, x0
            y0, y1 = y1, y0
        dx = x1 - x0
        dy = y1 - y0
        dir = -1 if dy < 0 else 1
        dy *= dir
        if dx != 0:
            y = y0
            p = 2 * dy - dx
            for i in range(dx + 1):
                cells.append((x0 + i, y))
                if p > 0:
                    y += dir
                    p -= 2 * dx
                p += 2 * dy
        return cells

    def _cells_between_v(self, x0, y0, x1, y1):
        cells = []
        if y0 > y1:
            x0, x1 = x1, x0
            y0, y1 = y1, y0
        dx = x1 - x0
        dy = y1 - y0
        dir = -1 if dx < 0 else 1
        dx *= dir
        if dy != 0:
            x = x0
            p = 2 * dx - dy
            for i in range(dy + 1):
                cells.append((x, y0 + i))
                if p >= 0:
                    x += dir
                    p -= 2 * dy
                p += 2 * dx
        return cells
    

    # def 
    
    # def world_to_grid(self, pos: Position, offset: Optional[Position]) -> tuple[int, int]:
    #     gx = round((pos.x - offset.x) / self.__size)
    #     gy = round((pos.y - offset.y) / self.__size)
    #     return gx, gy