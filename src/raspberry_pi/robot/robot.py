import threading
import time
import os
from typing import Tuple, Optional, List
from raspberry_pi.devices.rp2040 import RP2040
from raspberry_pi.devices.nano import NANO
from raspberry_pi.devices.lidar import Lidar
from raspberry_pi.data_structures.states import Position, CartPoint
from raspberry_pi.data_structures.maps import LocalMap, GlobalMap, OccupancyGrid
from raspberry_pi.config import ROBOT_CONFIG
from raspberry_pi.utils.logger import get_logger, timing_decorator

logger = get_logger(__name__)

class Robot:
    class ControlType:
        OFF = 'off'
        VELOCITY = 'velocity'
        POSITION = 'position'

    def __init__(self):
        # Initialize devices
        NANO.start()
        RP2040.start()
        Lidar.start()

        self.__control_thread = None
        self.__lock = threading.Lock()
        self.__stop_loop_event = threading.Event()
        self.__control_type: str = self.ControlType.OFF
        self.__target_position: Position = None
        self.__target_velocity: Tuple[float, float] = (0, 0)
    
        self.__mapping: bool = False
        self.__actual_position: Position = None
        self.__local_map: LocalMap = None
        self.__global_map: GlobalMap = None

    def __del__(self):
        NANO.stop()
        RP2040.stop()
        Lidar.stop()
    
    # @timing_decorator
    # def is_active(self):
    #     return self.__active
    
############# PUBLIC #############
    @timing_decorator
    def start(self):
        """ Starts the robot (on connection established).
            Currently does nothing.
        """
        pass
 
    @timing_decorator
    def stop(self):
        """ Stops the robot (on connection lost).
            Stops the control loop.
        """
        self.set_control('off')

    ## CONTROL
    @timing_decorator
    def get_control_type(self) -> str:
        """ Returns the current control type 
            ('off', 'velocity', 'position') 
        """
        with self.__lock:
            return self.__control_type
    
    @timing_decorator
    def set_control(self, ctype):
        """ Sets the control type
            ctype: ('off', 'manual', 'auto')
            internal state ('off', 'velocity', 'position')
        """
        if ctype in ['off', 'manual', 'auto']:
            with self.__lock:
                self.__target_velocity = (.0, .0)
                self.__target_position = None
                if ctype == 'off':
                    self.__control_type = self.ControlType.OFF
                elif ctype == 'manual':
                    self.__control_type = self.ControlType.VELOCITY
                elif ctype == 'auto':
                    self.__control_type = self.ControlType.POSITION
            if self.__control_type == self.ControlType.OFF:
                self.__stop_control_loop()
            else:
                self.__start_control_loop()
        else:
            raise ValueError(f"Invalid control type: {ctype}")

 
            
    @timing_decorator
    def set_target_velocity(self, linear, angular):
        self.__target_velocity = (linear, angular) # TODO lock??

    @timing_decorator
    def set_target_position(self, position):
        self.__target_position = position

    @timing_decorator
    def get_battery(self) -> int:
        """ Returns battery mV """
        with self.__lock:
            return NANO.get_battery()
        
    # MAPPING
    
    @timing_decorator
    def new_global_map(self, name: str) -> None:
        """ New global map
            does not require being active
            creates file for the map
        """
        with self.__lock:
            if self.__global_map is not None:
                raise Exception("Global map already initialized")
            self.__global_map = GlobalMap(name)
            self.RP2040.reset_odometry()
        os.makedirs(f"{ROBOT_CONFIG.MAP_FOLDER}/{name}", exist_ok=True)
            
    @timing_decorator
    def discard_global_map(self):
        """ Discards the global map 
            does not require being active
            if mapping, stops mapping
        """
        with self.__lock:
            if self.__mapping:
                self.__mapping = False
            self.__global_map = None
            RP2040.reset_odometry()
    
    @timing_decorator
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
    

    @timing_decorator
    def load_global_map(self):
        pass
        
    @timing_decorator
    def get_data(self, size_mm) -> Tuple[OccupancyGrid, List[CartPoint], Optional[Position]]:
        """ Returns data
            - global map: Occupancy Grid
            - local map: list of points (inside the grid frame)
            - position: global coordinates
        """
        global_map = OccupancyGrid(size_mm, ROBOT_CONFIG.GLOBAL_MAP_RESOLUTION)
        lidar_points = []
        position = None
        with self.__lock:
            # GLOBAL MAP
            if self.__global_map and self.__actual_position:
                global_map = self.__global_map.get_subsection(self.__actual_position, size_mm)
            # LOCAL MAP
            if self.__local_map:
                list_points = self.__local_map.get_cartesian_points(size_mm)
                lidar_points: Tuple[int, int] = [global_map.local_to_grid(p) for p in list_points]
            # POSITION
            if self.__actual_position:
                position = self.__actual_position
        return global_map, lidar_points, position
    
        
    @timing_decorator
    def get_global_map_subsection(self, size) -> Optional[OccupancyGrid]:
        """ Returns an occupancy grid
            returns None if no global map
        """
        with self.__lock:
            if self.__global_map is None or self.__actual_position is None:
                return None
            return self.__global_map.get_subsection(self.__actual_position, size)
    
    def get_local_points(self, size) -> Optional[List[CartPoint]]:
        """ Returns a local map
            returns None if no local map
        """
        with self.__lock:
            return self.__local_map.get_subsection(size) if self.__local_map else None
        
    @timing_decorator
    def reset_odometry(self):
        RP2040.reset_odometry()


############# PUBLIC ##############
    def __start_control_loop(self):
        with self.__lock:
            if self.__control_thread is None:
                self.__stop_loop_event.clear()
                self.__control_thread = threading.Thread(target=self.__loop, daemon=True)
                self.__control_thread.start()
                logger.info("Control thread started")
            else:
                logger.info("Control thread already started")

    def __stop_control_loop(self):
        with self.__lock:
            if self.__control_thread and self.__control_thread.is_alive():
                self.__stop_loop_event.set()
                self.__control_thread.join()
                self.__control_thread = None
                logger.info("Control thread stopped")
            else:
                logger.info("Control thread already stopped")
                            
    def __loop(self):
        """ Robot control loop
            Handles perception, planning and control
        """
        Lidar.start_scan()
        try:
            while not self.__stop_loop_event.is_set():
                logger.info("Loop running")
                # 🔒 LOCK 1 - Read shared state
                with self.__lock:
                    if self.__control_type == self.ControlType.OFF:
                        logger.info("Stopping control loop. (control OFF)")
                        break
                    global_map = self.__global_map  # Store reference safely
                    mapping_enabled = self.__mapping
                    control_type = self.__control_type  # Avoid repeated lock usage

                # 📡 Request sensor data (outside the lock)
                local_map = Lidar.produce_local_map()
                actual_pos = RP2040.get_position() if global_map else None

                # 🔒 LOCK 2 - Update shared state
                with self.__lock:
                    self.__local_map = local_map
                    self.__actual_position = actual_pos

                    # 🗺️ Expand global map (if mapping is enabled)
                    if global_map and mapping_enabled: 
                            global_map.expand(self.__actual_position, self.__local_map)
                    
                    # 🎯 Control logic
                    if control_type == self.ControlType.POSITION:
                        # TODO planning
                        target_position = self.__target_position
                        
                    elif control_type == self.ControlType.VELOCITY:
                        # TODO obstacle avoidance
                        target_velocity = self.__target_velocity
                
                # Control commands (outside the lock)
                if control_type == self.ControlType.POSITION:
                    RP2040.set_target_position(target_position)
                if control_type == self.ControlType.VELOCITY:
                    RP2040.set_target_velocity(*target_velocity)

                time.sleep(ROBOT_CONFIG.CONTROL_LOOP_INTERVAL)

        except Exception as e:
            logger.error(f"Robot loop error: {e}")
            
        finally:
            RP2040.stop_motors()
            Lidar.stop_scan()
