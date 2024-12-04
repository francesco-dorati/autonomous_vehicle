import numpy as np
import math
from raspberry_pi.structures.state import Position
from raspberry_pi.utils import Utils

# can be extended as localmap as abstarct class and Lidarmap, occupancy map as childs
class LocalMap:
    def __init__(self, scan):
        """
        Local Map
        A list of position taken at 1° steps

        :
            _scan (list): list of distances [0, 359]
        """
        self._scan = scan
    

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

    def is_segment_free(self, pos1: Position, pos2: Position) -> bool:
        """
        Is Segment Free
        Returns if the segment connecting the two points is free
        Positions are relative to the local map

        Args:
            pos1 (Position): starting point of the segment
            pos2 (Position): ending point of the segment

        Returns:
            bool: if segment is obstacle free
        """
        pass

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
    
    def get_scan(self) -> list:
        return self._scan.copy()




class GlobalMap:
    """
    Global Map
    Fixed resolution (100mm)
    Variable dimentions
    Occupancy grid
    (can be extended to a probability map returning what is the chance of a point to be occupied)
    """

    RESOLUTION = 100 # mm
    INITIAL_SIZE = 10000 # mm
    def __init__(self, robot_size):
        self._grid_size = self.INITIAL_SIZE
        self._grid = np.full((self.INITIAL_SIZE, self.INITIAL_SIZE), -1, dtype=int)
        self._origin_offset = (self.INITIAL_SIZE // 2, self.INITIAL_SIZE // 2)

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

    def expand(self, position: Position, local_map: LocalMap) -> None:
        """
        Expand
        Expands the map based on the local map and its position

        Args:
            position (Position): position of the local map
            local_map (LocalMap): local map of the envoironment
        """
        # check if local map fits in the current global map
            # if not expand the map

        scan = local_map.get_scan()
        for scan_angle_deg, scan_dist in enumerate(scan):
            scan_angle_mrad = Utils.deg_to_mrad(scan_angle_deg)
            obstacle_angle_mrad = Utils.normalize_mrad(position.th + scan_angle_mrad)
            obstacle_x = position.x + math.cos(obstacle_angle_mrad/1000)
            obstacle_y = position.y + math.sin(obstacle_angle_mrad/1000)
            obstacle_pos = Position(obstacle_x, obstacle_y, 0)
            self._expansion_ray(position, obstacle_pos)
    
    # get grid:  position, size -> array
    # get grid:  position, size -> 01110110101

    def _world_to_grid(self, pos: Position) -> tuple[int, int]:
        """
        World Position to Grid Position
        Position (0, 0) world is center of grid

        Args:
            pos (Position): position reference to the starting point

        Returns:
            int, int: index of the position
        """
        gx = int(pos.x / self.RESOLUTION) + self._origin_offset[0]
        gy = int(pos.y / self.RESOLUTION) + self._origin_offset[1]
        return gx, gy
    
    def _expansion_ray(self, robot_pos: Position, obstacle_pos: Position) -> None:
        """
        Obstacle Ray
        Set the line between robot and obstacle as free
            and the obstacle position as occupied

        Args:
            robot_pos (Position): position of the robot
            obstacle_pos (Position): position of the obstacle
        """