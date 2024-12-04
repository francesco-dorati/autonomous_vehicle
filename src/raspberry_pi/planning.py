from raspberry_pi.utils import timing_decorator
from raspberry_pi.structures.maps import GlobalMap, LocalMap
from raspberry_pi.structures.state import Position

class Planning:
    @staticmethod
    @timing_decorator
    def choose_exploration_goal(global_map: GlobalMap, position: Position):
        """
        Chooses goal position for mapping
        """
        pass

    @staticmethod
    @timing_decorator
    def path_planner(global_map: GlobalMap, local_map: LocalMap, current_pos: Position, goal_pos: Position) -> list[Position]:
        """
        Path Planner
        Finds a path from start to goal that avoids obstacles

        Args:
            global_map (GlobalMap): global map
            local_map (LocalMap): local map
            current_pos (Position): robot position
            goal_pos (Position): goal position

        Returns:
            List: path to follow
        """
        pass



    @staticmethod
    @timing_decorator
    def obstacle_avoidance(v_lin: int, v_th: int) -> tuple[int, int]:
        """
        Obstacle Avoidance
        Obstacle avoidance for manual control

        Args:
            v_lin (int): linear velocity of robot
            v_th (int): angular velocity of robot
            local_map (LocalMap): local map of envoironment
        Returns:
            v_lin (int): revised linear velocity of robot
            v_th (int): revised angular velocity of robot

        """
        pass