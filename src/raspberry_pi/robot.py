import threading
import time
import os
from typing import Tuple
from raspberry_pi.devices.rp2040 import RP2040
from raspberry_pi.devices.nano import NANO
from raspberry_pi.devices.lidar import Lidar
from raspberry_pi.data_structures.state import Position
from raspberry_pi.data_structures.maps import LocalMap, GlobalMap

class Robot:
    MAP_FOLDER = "./data/maps"
    def __init__(self):
        NANO.start()

        self.__thread = threading.Thread(target=self.__loop, daemon=True)
        self.__active: bool = False
        self.__lock = threading.Lock()
    
        self.__mapping: bool = False
        self.__actual_position: Position = None
        self.__local_map: LocalMap = None
        self.__global_map: GlobalMap = None

        self.__control_type: int = self.ControlType.OFF
        self.__target_position: Position = None
        self.__target_velocity: Tuple[float, float] = (0, 0)

    def __del__(self):
        NANO.stop()
        self.stop()

    def is_active(self):
        return self.__active
    
    def start(self):
        self.__active = True
        self.__thread.start()

    def stop(self):
        self.__active = False
        self.__thread.join()
        self.stop_control()

    class ControlType:
        OFF = 0
        VELOCITY = 1
        POSITION = 2

    def __loop(self):
        RP2040.start()
        
        Lidar.start()
        Lidar.start_scan()
        while self.__active:
            with self.__lock:
                ## LOCAL MAP
                self.__local_map = Lidar.produce_local_map()
                
                ## POSITION
                self.__actual_position = RP2040.get_position()

                ## MAPPING
                if self.__mapping:
                    self.__global_map.expand(self.__actual_position, self.__local_map)
                
                ## CONTROL
                if self.__control_type == self.ControlType.POSITION:
                    # PLANNING
                    pass
                elif self.__control_type == self.ControlType.VELOCITY:
                    # VELOCITY CONTROL
                    # TODO obstacle avoidance
                    RP2040.set_target_velocity(*self.__target_velocity)
                else:
                    # CONTROL OFF
                    RP2040.stop_motors()
            time.sleep(0.1)

        Lidar.stop_scan()
        Lidar.stop()
        RP2040.stop_motors()
        RP2040.stop()
        NANO.stop()
    
    def get_battery(self) -> int:
        """ Returns battery mV """
        with self.__lock:
            return NANO.get_battery()
    # MAPPING
    def new_global_map(self, name: str) -> None:
        """ New global map
            does not require being active
            creates file for the map
        """
        with self.__lock:
            if self.__global_map is not None:
                raise Exception("Global map already initialized")
            self.__global_map = GlobalMap(name)
        os.mkdirs(f"{Robot.MAP_FOLDER}/{name}", exist_ok=True)
        
    def discard_global_map(self):
        """ Discards the global map 
            does not require being active
            if mapping, stops mapping
        """
        with self.__lock:
            if self.__mapping:
                self.__mapping = False
            self.__global_map = None

    def save_global_map(self):
        """ Saves a copy of the global map
            does not require being active
            requires global map to be initialized
        """
        # both text and draw globalmap
        with self.__lock:
            if self.__global_map is None:
                raise Exception("Global map not initialized")
            g = self.__global_map.get_grid()
        # TODO save it

    def start_mapping(self):
        """ Starts mapping
            does not require being active 
            requires map to be initialized
        """
        with self.__lock:
            if self.__global_map is None:
                raise Exception("Global map not initialized")
            self.__mapping = True

    def stop_mapping(self):
        """ Stops mapping
            does not require being active
        """
        with self.__lock:
            self.__mapping = False

    def load_global_map(self):
        pass
    
    def get_global_map(self):
        pass

    def reset_odometry(self):
        pass

    # CONTROL
    def get_control_type(self) -> int:
        with self.__lock:
            return self.__control_type
        
    def stop_control(self):
        with self.__lock:
            self.__control_type = self.ControlType.OFF
            self.__target_velocity = (.0, .0)
            self.__target_position = None
 
    def set_target_velocity(self, linear, angular):
        with self.__lock:
            self.__control_type = self.ControlType.VELOCITY
            self.__target_velocity = (linear, angular)
            self.__target_position = None
        
    def set_target_position(self, position):
        with self.__lock:
            self.__control_type = self.ControlType.POSITION
            self.__target_velocity = (.0, .0)
            self.__target_position = position
        