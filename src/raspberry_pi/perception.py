from raspberry_pi.utils import timing_decorator
from raspberry_pi.structures.state import Position, State, Velocity
from raspberry_pi.structures.maps import LocalMap, GlobalMap

class Perception:
    @staticmethod
    @timing_decorator
    def visual_odometry(prev_position: Position, lidar_data) -> Position:
        """
        Visual odometry
        Compares two lidar frames to compute their distance
        The distance is then applied to the previous position to compute a new position

        Args:
            prev_position (Position): position of the previous frame
            lidar_data (_type_): _description_

        Returns:
            Position: position of the new frame
        """
        pass

    @staticmethod
    @timing_decorator
    def position_filter(visual_pos: Position, encoder_pos: Position) -> Position:
        """
        Position filter
        Merges the two positions

        Args:
            visual_pos (Position): visual odometry position
            encoder_pos (Position): encoder odometry position

        Returns:
            Position: merged position
        """
        pass

    @staticmethod
    @timing_decorator
    def map_construction(position: Position, local_map: LocalMap, global_map: GlobalMap) -> GlobalMap:
        """
        Map Construction
        Contruct the global map

        Args:
            position (Position): position of the local frame
            local_map (LocalMap): local frame
            global_map (GlobalMap): old global map

        Returns:
            GlobalMap: updated global map
        """
        global_map.expand(position, local_map)
        return global_map