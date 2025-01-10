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
import numpy as np

from raspberry_pi.data_structures.state import Position
from raspberry_pi.data_structures.lidar_scan import LidarScan
from raspberry_pi.utils import Utils, timing_decorator


# can be extended as localmap as abstarct class and lidarmap, occupancy map as childs
class LocalMap:
    def __init__(self, scan: LidarScan):
        """
        Local Map
        A list of position taken at 1° steps
        _scan (list): list of distances [0, 359]

        Arguments:
            Lidar.Scan: scan object 
        """
        self._scan = scan.get_copy()

    def __repr__(self):
        return f"local_map()"

    @timing_decorator
    def get_iterable(self) -> list: # TODO oppure get list che ritorna una lista ??
        return iter(enumerate(self._scan.copy()))
    
    @timing_decorator
    def draw(self, filename) -> None:
        import matplotlib.pyplot as plt
        path_name = f"../img/{filename}.png"
        x_points = []
        y_points = []
        for angle, distance in enumerate(self._scan):
            if distance == 0: 
                continue
            # Convert angle from degrees to radians for trigonometric functions
            angle_rad = np.radians(360 - angle)
            # Skip points with invalid or zero distance
            if distance > 0:
                x = distance * np.cos(angle_rad)
                y = distance * np.sin(angle_rad)
                x_points.append(x)
                y_points.append(y)
        # Plotting
        plt.figure(figsize=(8, 8))
        plt.scatter(x_points, y_points, s=1, color='blue')  # Smaller marker for a denser plot
        # Robot position
        plt.plot(0, 0, 'ro', label="Robot Position")  # Red dot for robot location
        plt.arrow(0, 0, 500, 0, head_width=100, head_length=80, fc='red', ec='red', label="Heading")
        plt.gca().invert_yaxis()  # Invert y-axis to make the plot look correct
        plt.title("LIDAR Scan")
        plt.xlabel("X (mm)")
        plt.ylabel("Y (mm)")
        plt.axis('equal')  # Ensures aspect ratio is equal for X and Y axes
        plt.grid(True)
        plt.savefig(path_name, format='png')
        plt.close() 

    @timing_decorator
    def get_dist(self, angle: int) -> int:
        """
        Get Dist
        Get the distance at a specified angle

        Args:
            angle (int): angle where to check distance

        Returns:
            int: distance in mm
        """
        return self._scan[angle]
    
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

    INITIAL_SIZE_MM =  10000
    RESOLUTION = 100 # mm
    def __init__(self):
        self._grid_size = (self.INITIAL_SIZE_MM // self.RESOLUTION)
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
        scan = local_map.get_iterable()
        for scan_angle_deg, scan_dist in scan:
            if scan_dist == 0:
                continue

            scan_angle_mrad = Utils.deg_to_mrad(scan_angle_deg)

            obstacle_angle_mrad = Utils.normalize_mrad(position.th + scan_angle_mrad)
            obstacle_x = position.x + scan_dist*math.cos(obstacle_angle_mrad/1000)
            obstacle_y = position.y + scan_dist*math.sin(obstacle_angle_mrad/1000)
            obstacle_pos = Position(obstacle_x, obstacle_y, 0)
            if not self._is_inside(obstacle_pos):
                self._expand_grid()

            self._ray_cast(position, obstacle_pos)
    
    @timing_decorator
    def draw(self, filename) -> None:
        import matplotlib.pyplot as plt
        from matplotlib.colors import ListedColormap

        path_name = f"../img/{filename}.png"
        # Plotting
        cmap = ListedColormap(['gray', 'white', 'black'])
        bounds = [-1.5, -0.5, 0.5, 1.5]  # Boundaries for each value
        norm = plt.matplotlib.colors.BoundaryNorm(bounds, cmap.N)

        plt.figure(figsize=(8, 8))
        plt.imshow(self._grid.T, cmap=cmap, norm=norm, origin='upper')
        # Robot position
        y, x = self.origin()
        plt.plot(y, x, 'ro', label="Robot Position")  # Red dot for robot location
        plt.arrow(y, x, 3, 0, head_width=1, head_length=1, fc='red', ec='red', label="Heading")
        plt.arrow(y, x, 0, 3, head_width=1, head_length=1, fc='green', ec='green', label="Heading")
        plt.gca().invert_yaxis()  # Invert y-axis to make the plot look correct
        plt.title("LIDAR Scan")
        plt.xlabel("X (mm)")
        plt.ylabel("Y (mm)")
        plt.axis('equal')  # Ensures aspect ratio is equal for X and Y axes
        plt.grid(True)
        plt.savefig(path_name, format='png')
        plt.close() 

    # get grid:  position, size -> array (to send to dev)
    # get grid:  position, size -> 01110110101



    def _expand_grid(self) -> None:
        new_size = (self._grid_size*2)
        new_grid = np.full((new_size, new_size), -1, dtype=int)
        off = (new_size - self._grid_size) // 2

        new_grid[off:off+self._grid_size, off:off+self._grid_size] = self._grid
        self._grid_size = new_size
        self._grid = new_grid

    def _world_to_grid(self, pos: Position) -> tuple[int, int]:
        """
        World Position to Grid Position
        Position (0, 0) world is center of grid

        Args:
            pos (Position): position reference to the starting point

        Returns:
            int, int: index of the position
        """
        gx = round(pos.x / self.RESOLUTION) + (self._grid_size // 2)
        gy = round(pos.y / self.RESOLUTION) + (self._grid_size // 2)
        return gx, gy
    
    def _is_inside(self, p: Position):
        gx, gy = self._world_to_grid(p)
        inside_x = (gx >= 0 and gx < self._grid_size)
        inside_y = (gy >= 0 and gy < self._grid_size)
        inside = inside_x and inside_y
        return inside
        
    def _set_unknown(self, gx, gy) -> None:
        self._grid[gx][gy] = -1
    
    def _set_free(self, gx, gy) -> None:
        self._grid[gx][gy] = 0
    
    def _set_occupied(self, gx, gy) -> None:
        self._grid[gx][gy] = 1 

    def _ray_cast(self, robot_pos: Position, obstacle_pos: Position) -> None:
        """
        Ray Cast
        Set the line between robot and obstacle as free
            and the obstacle position as occupied

        Args:
            robot_pos (Position): position of the robot
            obstacle_pos (Position): position of the obstacle
        """
        xr, yr = self._world_to_grid(robot_pos)
        xo, yo = self._world_to_grid(obstacle_pos)
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