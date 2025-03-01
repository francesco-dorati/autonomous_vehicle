import threading
import time
import os
import numpy as np
from typing import Tuple, Optional, List
from raspberry_pi.devices.rp2040 import RP2040
from raspberry_pi.devices.nano import NANO
from raspberry_pi.devices.lidar import Lidar
from raspberry_pi.data_structures.states import Position, CartPoint
from raspberry_pi.data_structures.maps import LocalMap, GlobalMap, OccupancyGrid
from raspberry_pi.config import ROBOT_CONFIG
from raspberry_pi.utils.logger import get_logger, timing_decorator
from raspberry_pi.utils.utils import Utils
from raspberry_pi.utils.drawer import Drawer
import open3d as o3d

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
        self.__prev_local_map: List[CartPoint] = []
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
        RP2040.stop_motors()
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
                self.__local_map = None
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
        
    ### MAPPING ###
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
        RP2040.reset_position()
        os.makedirs(f"{ROBOT_CONFIG.MAPS_FOLDER}/{name}", exist_ok=True)
            
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
        RP2040.reset_position()
    
    @timing_decorator
    def save_global_map(self):
        """ Saves a copy of the global map
            does not require being active
            requires global map to be initialized
        """
        # both text and draw globalmap
        with self.__lock:
            logger.debug("Copy map for saving")
            if self.__global_map is None:
                raise Exception("Global map not initialized")
            grid: OccupancyGrid = self.__global_map.get_copy()
        logger.debug("Saving map")
        Drawer.save_global_map(self.__global_map.name, grid)
    
    @timing_decorator
    def load_global_map(self):
        pass

    @timing_decorator
    def start_mapping(self):
        logger.debug("Starting mapping")
        with self.__lock:
            self.__mapping = True

    @timing_decorator
    def stop_mapping(self):
        with self.__lock:
            self.__mapping = False
        
    @timing_decorator
    def get_data(self, size_mm) -> Tuple[OccupancyGrid, List[Tuple[int, int]], Optional[Position]]:
        """ Returns data
            - global map: Occupancy Grid
            - local map: list of points (inside the grid frame)
            - position: global coordinates
        """
        global_map: OccupancyGrid = OccupancyGrid(size_mm)
        lidar_grid_points = []
        position = None
        logger.info("ROBOT get data")
        with self.__lock:
            # GLOBAL MAP
            if self.__global_map and self.__actual_position:
                global_map = self.__global_map.get_subsection(size_mm, self.__actual_position)

            # LOCAL MAP
            if self.__local_map:
                local_points: List[CartPoint] = self.__local_map.get_cartesian_points(size_mm)
                lidar_grid_points: List[Tuple[int, int]] = global_map.local_to_grid(local_points)
            
            # POSITION
            if self.__actual_position:
                position = self.__actual_position

        # global_map = global_map.get_grid()
        return global_map, lidar_grid_points, position
    
    ### END MAP ###

    @timing_decorator
    def reset_odometry(self):
        RP2040.reset_odometry()
############# END PUBLIC ##############

############# PRIVATE ##############
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

    def __visual_odometry(self, current_points: List[CartPoint], prev_points: List[CartPoint]):     
        try:
            t = time.time()
            # Converti entrambe le scansioni in point cloud 3D (aggiungendo z=0)
            curr_pts = np.array([[pt.x, pt.y] for pt in current_points], dtype=np.float64)
            prev_pts = np.array([[pt.x, pt.y] for pt in prev_points], dtype=np.float64)
  
            if prev_pts.size > 0 and curr_pts.size > 0:
                pts_prev_3d = np.hstack([prev_pts, np.zeros((prev_pts.shape[0], 1))])
                pts_curr_3d = np.hstack([curr_pts, np.zeros((curr_pts.shape[0], 1))])
                
                pc_prev = o3d.geometry.PointCloud()
                pc_curr = o3d.geometry.PointCloud()
                pc_prev.points = o3d.utility.Vector3dVector(pts_prev_3d)
                pc_curr.points = o3d.utility.Vector3dVector(pts_curr_3d)
                
                # Imposta una soglia per ICP (da adattare alle unità del tuo sistema)
                threshold = 5.0
                trans_init = np.eye(4)
                reg_result = o3d.pipelines.registration.registration_icp(
                    pc_curr, pc_prev, threshold, trans_init,
                    o3d.pipelines.registration.TransformationEstimationPointToPoint()
                )
                transformation = reg_result.transformation
                logger.info(f"ICP transformation:\n{transformation}")
                # Estrai la traslazione stimata (in x, y, z)
                delta_translation = transformation[:3, 3]
                logger.info(f"Estimated translation (x,y,z): {delta_translation}")
                
                dt = time.time() - t
                logger.info(f"ICP time: {dt}")
 
                return delta_translation
            else:
                logger.warning("Scansioni ICP vuote!")

        except Exception as icp_e:
            logger.error(f"ICP localization error: {icp_e}")

    def __loop(self):# TODO draw n lidar maps
        """ Robot control loop
            Handles perception, planning and control
        """
        i = 0
        Lidar.start_scan()
        try:
            while not self.__stop_loop_event.is_set():
                logger.debug("LOOP start")
                # 🔒 LOCK 1 - Read shared state
                with self.__lock:
                    logger.debug("LOOP acquired first lock")
                    if self.__control_type == self.ControlType.OFF:
                        logger.info("LOOP stopping control loop. (control OFF)")
                        break
                    global_map: GlobalMap = self.__global_map  # Store reference safely
                    mapping_enabled: bool = self.__mapping
                    control_type: str = self.__control_type  # Avoid repeated lock usage

                logger.debug(f"LOOP released first lock")

                # 📡 Request sensor data (outside the lock)
                local_map = Lidar.produce_local_map()
                actual_pos = RP2040.get_position() if global_map else None

                logger.debug("LOOP retreived data from external devices")
                logger.debug(f"LOOP: control_type: {control_type}, mapping: {mapping_enabled}, position: {actual_pos}")
                
                # IMPLEMENTAZIONE LOCALIZZAZIONE CON ICP
                # Se è disponibile una scansione precedente, usa ICP per stimare la trasformazione.
                if local_map.get_size() > 0:
                    logger.info("Local map available for ICP localization.")
                    cart_points = local_map.get_cartesian_points()
                    if len(self.__prev_local_map) > 0:
                        logger.debug("Previous local map available for ICP localization.")
                        # logger.debug(f"cart points: {cart_points} ")
                        # logger.debug(f"old points: {self.__prev_local_map}")
                        delta_translation = self.__visual_odometry(cart_points, self.__prev_local_map)
                    else:
                        logger.debug("No previous local map available for ICP localization.")
                    
                    self.__prev_local_map = cart_points
                else:
                    logger.warning("No local map available for ICP localization.")
                    

                # 🔒 LOCK 2 - Update shared state
                with self.__lock:
                    logger.debug("LOOP acquired second lock")

                    self.__local_map = local_map
                    self.__actual_position = actual_pos

                    # 🗺️ Expand global map (if mapping is enabled)
                    if global_map and mapping_enabled: 
                            logger.debug("LOOP expanding global map")
                            global_map.expand(self.__actual_position, self.__local_map)
                    
                    # 🎯 Control logic
                    target_position = None
                    target_velocity = (0, 0)
                    if control_type == self.ControlType.POSITION:
                        # TODO planning
                        target_position = self.__target_position
                        
                    elif control_type == self.ControlType.VELOCITY:
                        # TODO obstacle avoidance
                        target_velocity = self.__target_velocity

                logger.debug("LOOP Second lock released")
                logger.debug(f"LOOP: target_position: {target_position}, target_velocity: {target_velocity}")

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
