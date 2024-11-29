from raspberry_pi.structures.state import Position

class LocalMap:
    def __init__(self, robot_size):
        pass

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

class GlobalMap:
    """
    Global Map
    Fixed resolution (100mm)
    Variable dimentions
    Occupancy grid
    (can be extended to a probability map returning what is the chance of a point to be occupied)
    """
    def __init__(self, robot_size):
        pass

    def is_position_free(self, pos: Position) -> bool:
        """
        Is Position Free
        Returns if position is obstacle free

        Args:
            pos (Position): global position

        Returns:
            bool: if position is free
        """
        pass

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
    
        pass