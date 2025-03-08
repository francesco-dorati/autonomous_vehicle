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

Naming Conventions:
    - world: refers to the external environment
    - local: refers to the robot's position
    all positions refers to world
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

        def get_copy(self) -> List[PolarPoint]:
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
    """
        Occupancy Grid
        __origin_world (CartPoint): origin of the grid in world coordinates


        - local refers to the grid's origin
        - grid refers to the cell
        - world refers to the center of the world
        ()
    """
    def __init__(self, size_mm: int):
        self.__size_mm: int = size_mm
        self.__grid_size: int = Utils.dist_to_grid(size_mm)
        self.__grid = None
        self.__origin_world: Position = None

    def is_set(self):
        return self.__grid is not None
        
    def set_origin_world(self, point: CartPoint):
        self.__origin_world = point

    # def set_from(self, other):
    #     if self.__grid is not None:
    #         logger.error("Trying to set grid from a non empty grid")
    #         raise Exception("Trying to set grid from a non empty grid")
  
    #     origin: Position = self.get_origin_world()
    #     other_origin: Position = other.get_origin_world()
    #     if not origin or not other_origin or origin != other_origin:
    #         logger.error("Trying to set grid from a grid without origin")
    #         raise Exception("Trying to set grid from a grid without origin")
        
    #     self.__grid = np.full((self.__grid_size, self.__grid_size), -1, dtype=int)

    #     if self.__grid_size >= other.get_grid_size():
    #         # Our grid is larger: center the other grid into ours.
    #         other_grid_size = other.get_grid_size()
    #         off = (self.__grid_size - other_grid_size) // 2
    #         self.__grid[off:off+other_grid_size, off:off+other_grid_size] = np.copy(other._OccupancyGrid__grid)
    #     else:
    #         # Our grid is smaller: extract the central subgrid of the other grid that fits into ours.
    #         other_grid_size = other.get_grid_size()
    #         off = (other_grid_size - self.__grid_size) // 2
    #         self.__grid = np.copy(other._OccupancyGrid__grid[off:off+self.__grid_size, off:off+self.__grid_size])

    def set_from(self, other):
        if self.__grid is not None:
            logger.error("Trying to set grid from a non empty grid")
            raise Exception("Trying to set grid from a non empty grid")
        if other.__grid is None:
            logger.error("Trying to set grid from an empty grid")
            self.__grid = None
            return
    
        # Get origins (world coordinates) from both grids.
        origin: CartPoint = self.get_origin_world()
        other_origin: CartPoint = other.get_origin_world()
        if not origin or not other_origin:
            logger.error("One of the grids does not have a world origin set")
            raise Exception("One of the grids does not have a world origin set")
        
        # Initialize our grid to unknown (-1)
        self.__grid = np.full((self.__grid_size, self.__grid_size), -1, dtype=int)

        # Resolution (mm per grid cell)
        resolution = ROBOT_CONFIG.GLOBAL_MAP_RESOLUTION

        # Compute displacement in world coordinates (mm) between our origin and other's origin.
        dx_world = other_origin.x - origin.x
        dy_world = other_origin.y - origin.y

        # Convert the displacement into grid cell offsets (round to nearest integer).
        dx_cells = round(dx_world / resolution)
        dy_cells = round(dy_world / resolution)

        # Get the size (number of cells) of the other grid.
        other_grid_size = other.get_grid_size()

        # Compute the base insertion offset:
        # - Our grid's center is at (self.__grid_size//2, self.__grid_size//2)
        # - Other grid's center is at (other_grid_size//2, other_grid_size//2)
        # Then add the relative offset (dx_cells, dy_cells).
        insert_x = (self.__grid_size // 2) - (other_grid_size // 2) + dx_cells
        insert_y = (self.__grid_size // 2) - (other_grid_size // 2) + dy_cells

        # Determine the overlapping region between our grid and the other grid.
        # In our grid, valid indices: [0, self.__grid_size)
        self_x_start = max(0, insert_x)
        self_x_end   = min(self.__grid_size, insert_x + other_grid_size)
        self_y_start = max(0, insert_y)
        self_y_end   = min(self.__grid_size, insert_y + other_grid_size)
        
        # Corresponding indices in the other grid:
        other_x_start = max(0, -insert_x)
        other_x_end   = other_x_start + (self_x_end - self_x_start)
        other_y_start = max(0, -insert_y)
        other_y_end   = other_y_start + (self_y_end - self_y_start)

        # Copy the overlapping region from the other grid into our grid.
        self.__grid[self_x_start:self_x_end, self_y_start:self_y_end] = \
            np.copy(other._OccupancyGrid__grid[other_x_start:other_x_end, other_y_start:other_y_end])

    def set_free(self, gx, gy):
        if self.__grid is None:
            self.__grid = np.full((self.__grid_size, self.__grid_size), -1, dtype=int)
        self.__grid[gx][gy] = 0
    def set_unknown(self, gx, gy):
        if self.__grid is None:
            self.__grid = np.full((self.__grid_size, self.__grid_size), -1, dtype=int)
        self.__grid[gx][gy] = -1
    def set_occupied(self, gx, gy):
        if self.__grid is None:
            self.__grid = np.full((self.__grid_size, self.__grid_size), -1, dtype=int)
        self.__grid[gx][gy] = 1
    
    def get_size_mm(self) -> int:
        return self.__size_mm
    def get_grid_size(self) -> int:
        return self.__grid_size
    def get_origin_world(self) -> CartPoint:
        return self.__origin_world
    def get(self, gx, gy) -> int:
        if not (0 <= gx < self.__grid_size and 0 <= gy < self.__grid_size):
            return -1
        if self.__grid is None:
            return -1
        return self.__grid[gx][gy]
    
    def get_bytes(self) -> bytes:
        """ get the bytes of the grid (shifted by 1) """
        if self.__grid is None:
            return b""
        return self.__grid.astype(np.int8).tobytes()
    
    def get_string(self, row_sep: str = ";") -> str:
        if self.__grid is None:
            return "-"
        return row_sep.join([" ".join(map(str, row)) for row in self.__grid])
    
    def get_copy(self):
        copy_grid = OccupancyGrid(self.__size_mm)
        copy_grid.set_origin_world(self.__origin_world) 
        if self.__grid is not None:
            copy_grid.__grid = np.copy(self.__grid)  # Deep copy of the grid array
        return copy_grid
    def get_grid(self) -> np.ndarray:
        return self.__grid
    def origin_grid(self):
        return ((self.__grid_size // 2), (self.__grid_size // 2))

    def are_inside(self, local_points: List[CartPoint]) -> bool:
        logger.debug("Are inside?")
        grid_points = self.local_to_grid(local_points)
        ok = True
        for gx, gy in grid_points:
            if not ok:
                break
            inside_x = (gx >= 0 and gx < self.__grid_size)
            inside_y = (gy >= 0 and gy < self.__grid_size)
            ok = ok and inside_x and inside_y
        logger.debug(f"Are inside {ok}")
        return ok

    def local_to_grid(self, local_points: List[CartPoint]) -> List[Tuple[int, int]]:
        """Converts a local point to grid point."""
        grid_points = []
        for point in local_points:
            gx = int(round(point.x / ROBOT_CONFIG.GLOBAL_MAP_RESOLUTION) + (self.__grid_size // 2))
            gy = int(round(point.y / ROBOT_CONFIG.GLOBAL_MAP_RESOLUTION) + (self.__grid_size // 2))
            grid_points.append((gx, gy))
        return grid_points
    
    # def local_to_world(self, position: Position, local_points: List[CartPoint]) -> List[CartPoint]:
    #     pass
    # def other_to_local(self, other_position: Position, other_points: List[CartPoint]) -> List[CartPoint]:
    #     other_theta_rad = other_position.th / 1000
    #     world_points = []
    #     for point in other_points:
    #         x = other_position.x + point.x*np.cos(other_theta_rad) - point.y*np.sin(other_theta_rad)
    #         y = other_position.y + point.x*np.sin(other_theta_rad) + point.y*np.cos(other_theta_rad)
    #         world_points.append(CartPoint(x, y))
    #     return world_points
    
    def ray_cast(self, local_point: CartPoint, local_obstacle_points: List[CartPoint]) -> None:
        cast_origin: Tuple[int, int] =  self.local_to_grid([local_point])[0]
        grid_points: List[Tuple[int, int]] = self.local_to_grid(local_obstacle_points)
        for obstacle_grid_point in grid_points:
            cells_between = self.__cells_between(cast_origin, obstacle_grid_point)
            for grid_point in cells_between:
                self.set_free(*grid_point)
            self.set_occupied(*obstacle_grid_point)

    def __cells_between(self, origin: Tuple[int, int], obstacle: Tuple[int, int]) -> List[Tuple[int, int]]:
        x0, y0 = origin
        x1, y1 = obstacle
        if abs(x1-x0) > abs(y1-y0):
            # HORIZONTAL
            cells = self.__cells_between_h(x0, y0, x1, y1)
        else:
            # VERTICAL
            cells = self.__cells_between_v(x0, y0, x1, y1)
        return cells
    def __cells_between_h(self, x0, y0, x1, y1):
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
    def __cells_between_v(self, x0, y0, x1, y1):
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
    
    # def local_to_world(self, position: Position, local_points: List[CartPoint]) -> List[CartPoint]:
    #     global_points = []
    #     robot_th_rad = position.th / 1000
    #     for lx, ly in local_points:
    #         x = position.x + lx*np.cos(robot_th_rad) - ly*np.sin(robot_th_rad)
    #         y = position.y + lx*np.sin(robot_th_rad) + ly*np.cos(robot_th_rad)
    #         global_points.append(CartPoint(x, y))
    #     return global_points


# can be extended as localmap as abstarct class and lidarmap, occupancy map as childs
class LocalMap:
    def __init__(self, scan: List[PolarPoint]):
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

    def get_size(self):
        return len(self.__scan)
 
    @timing_decorator
    def get_polar_points(self, section_radius: int = None) -> List[PolarPoint]:
        """Get polar coordinates of the scan            
        """
        if section_radius is None:
            return [coord for coord in self.__scan]
        return [coord for coord in self.__scan if coord.distance <= section_radius]
    
    @timing_decorator
    def get_cartesian_points(self, map_position: Position | None = None, section_size: int = None) -> List[CartPoint]:
        """Get cartesian coordinates of the scan (based on the center of the local map)
        Returns:
            List[CartPoint]: list cartesian points in local coordinates (refers to position of the robot)
        """
        # logger.debug("Get Cartesian Points")
        max_dist = (section_size//2)*ROBOT_CONFIG.GLOBAL_MAP_RESOLUTION if section_size else None
        points = [] 
        for p in self.__scan:
            # Convert angle from degrees to radians for trigonometric functions
            try:
                angle_rad: float = (p.th + map_position.th)/1000.0 if map_position else p.th/1000.0      # p.th already normalized
                if p.r > 0 and (max_dist is None or p.r < section_size):
                    cart_p = CartPoint(
                        p.r*np.cos(angle_rad) + (map_position.x if map_position else 0), 
                        p.r*np.sin(angle_rad) + (map_position.y if map_position else 0)
                    )
                    points.append(cart_p)
            except Exception as e:
                logger.error(f"Error in get_cartesian_points {e}")
                raise Exception("Error in get_cartesian_points")
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
        self.__grid = OccupancyGrid(ROBOT_CONFIG.GLOBAL_MAP_SIZE_MM)
        self.__grid.set_origin_world(CartPoint(0, 0))
        
        # self.origin = lambda: ((self._grid_size // 2), (self._grid_size // 2))
    
    def __repr__(self):
        return f"global_map(size={self.__grid.get_grid_size()})"

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
        return self._grid.get(gx, gy) == 0

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
    def expand(self, robot_position: Position, local_map: LocalMap) -> None:
        """
        Expand
        Expands the map based on the local map and its position

        Args:
            position (Position): position of the local map
            local_map (LocalMap): local map of the envoironment
        """
        logger.debug("Expanding grid")
        robot_point: CartPoint = robot_position.get_point()

        logger.debug("Expanding grid 1")
        obstacle_points: List[CartPoint] = local_map.get_cartesian_points(robot_position) # local based on robot position
        
        # logger.debug("Expanding grid 2")
        # obstacle_points_global: List[CartPoint] = self.__grid.other_to_local(robot_position, local_points)
        
        logger.debug("Expanding grid 3")
        while not self.__grid.are_inside(obstacle_points):
            logger.debug("Grid too small, expanding it")
            self.__expand_grid()

        logger.debug("Expanding grid 4")
        self.__grid.ray_cast(robot_point, obstacle_points)

        logger.debug("Expanding grid OK")
        
        # #####
        # for point in global_points:
        #     if not self._is_inside(point):
        #     gx, gy = self._world_to_grid(point)
        #     self._set_free(gx, gy)

        # for lx, ly in local_points:
        #     robot_th_rad = position.th / 1000
        #     obs_x = position.x + lx*np.cos(robot_th_rad) - ly*np.sin(robot_th_rad)
        #     obs_y = position.y + lx*np.sin(robot_th_rad) + ly*np.cos(robot_th_rad)
        #     obs_point = CartPoint(obs_x, obs_y) # global coordinates
        #     robot_point = position.get_point()
        #     if not self._is_inside(obs_point):
        #         self._expand_grid()
        #     self._ray_cast(robot_point, obs_point)

    def get_copy(self) -> OccupancyGrid:
        return self.__grid.get_copy()
    
    def get_subsection(self, origin_world: CartPoint, size_mm: int, ) -> OccupancyGrid:
        """
        Get Subsection
        Returns a subsection of the grid based on the position and size

        Args:
            position (Position): center of the subsection
            size (int): size of the subsection

        Returns:
            OccupancyGrid: subsection of the grid
        """
        # ERRRORR HERE
        subsection = OccupancyGrid(size_mm)
        logger.info("Occupancy grid")

        subsection.set_origin_world(origin_world)
        logger.info("Occupancy grid, set origin")
        
        subsection.set_from(self.__grid)
        logger.info("Occupancy grid, set from")
        return subsection

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



    def __expand_grid(self) -> None:
        """ Expand grid with one double the size """
        new_size_mm = self.__grid.get_size_mm() * 2
        new_grid = OccupancyGrid(new_size_mm)
        new_grid.set_origin_world(self.__grid.get_origin_world())
        new_grid.set_from(self.__grid)
        self.__grid = new_grid


    # def _world_to_grid(self, p: CartPoint) -> tuple[int, int]:
        # """
        # World Position to Grid Position
        # Position (0, 0) world is center of grid

        # Args:
        #     pos (Position): position reference to the starting point

        # Returns:
        #     int, int: index of the position
        # """
        # gx = round(p.x / self.RESOLUTION) + (self._grid_size // 2)
        # gy = round(p.y / self.RESOLUTION) + (self._grid_size // 2)
        # return gx, gy
    
    # def _is_inside(self, p: CartPoint):
    #     gx, gy = self._world_to_grid(p)
    #     inside_x = (gx >= 0 and gx < self._grid_size)
    #     inside_y = (gy >= 0 and gy < self._grid_size)
    #     inside = inside_x and inside_y
    #     return inside
        
    # def _get(self, p: CartPoint) -> int:
    #     gx, gy = self._world_to_grid(p)
    #     return self._grid[gx][gy]
    
    # def _set_unknown(self, gx, gy) -> None:
    #     self._grid[gx][gy] = -1
    
    # def _set_free(self, gx, gy) -> None:
    #     self._grid[gx][gy] = 0
    
    # def _set_occupied(self, gx, gy) -> None:
    #     self._grid[gx][gy] = 1 

    # def _ray_cast(self, robot_point: CartPoint, obs_point: CartPoint) -> None:
    #     """
    #     Ray Cast
    #     Set the line between robot and obstacle as free
    #         and the obstacle position as occupied

    #     Args:
    #         robot_pos (Position): position of the robot
    #         obstacle_pos (Position): position of the obstacle
    #     """
    #     xr, yr = self._world_to_grid(robot_point)
    #     xo, yo = self._world_to_grid(obs_point)
    #     cells_between = self._cells_between(xr, yr, xo, yo)
    #     for gx, gy in cells_between:
    #         self._set_free(gx, gy)
    #     self._set_occupied(xo, yo)
    
    # def _cells_between(self, x0, y0, x1, y1) -> list:
    #     """
    #     Cells Between
    #     Returns:
    #         list: list of (gx, gy), including initial and final values [(gx0, gy0), ..., (gx1, gy1)]
    #     """
    #     if abs(x1-x0) > abs(y1-y0):
    #         # HORIZONTAL
    #         cells = self._cells_between_h(x0, y0, x1, y1)
    #     else:
    #         # VERTICAL
    #         cells = self._cells_between_v(x0, y0, x1, y1)
    #     return cells
    
    # def _cells_between_h(self, x0, y0, x1, y1):
    #     cells = []
    #     if x0 > x1:
    #         x0, x1 = x1, x0
    #         y0, y1 = y1, y0
    #     dx = x1 - x0
    #     dy = y1 - y0
    #     dir = -1 if dy < 0 else 1
    #     dy *= dir
    #     if dx != 0:
    #         y = y0
    #         p = 2 * dy - dx
    #         for i in range(dx + 1):
    #             cells.append((x0 + i, y))
    #             if p > 0:
    #                 y += dir
    #                 p -= 2 * dx
    #             p += 2 * dy
    #     return cells

    # def _cells_between_v(self, x0, y0, x1, y1):
    #     cells = []
    #     if y0 > y1:
    #         x0, x1 = x1, x0
    #         y0, y1 = y1, y0
    #     dx = x1 - x0
    #     dy = y1 - y0
    #     dir = -1 if dx < 0 else 1
    #     dx *= dir
    #     if dy != 0:
    #         x = x0
    #         p = 2 * dx - dy
    #         for i in range(dy + 1):
    #             cells.append((x, y0 + i))
    #             if p >= 0:
    #                 x += dir
    #                 p -= 2 * dy
    #             p += 2 * dx
    #     return cells
    

    # def 
    
    # def world_to_grid(self, pos: Position, offset: Optional[Position]) -> tuple[int, int]:
    #     gx = round((pos.x - offset.x) / self.__size)
    #     gy = round((pos.y - offset.y) / self.__size)
    #     return gx, gy