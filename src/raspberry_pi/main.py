"""
    Robot Main Class

    INPUTS

    Main Connection - (TCP Socket)
    
        PING
        ping command
        "PNG" -> "P <status> <battery_mv>"
        status: status code (to be defined)
        battery_mv: battery voltage in millivolts

        MANUAL
        activate/deactivate manual control
        "MAN <code>" -> "OK <manual_port>"
        code:   0   off
                1   on
        manual_port: manual receiver port

        MAPPING
        activate/deactivate mapping
        "MAP <code>" -> "OK"
        code:   0 off
                1 on



        MAPPING
            AUTONOMOUS
            MANUAL

        LOCALIZE
            AUTONOMOUS
            MANUAL
        
        MOVE
            AUTONOMOUS
                - go to goal
                - follow path
            MANUAL


    





"""



import time
import socket
from enum import Enum
from threading import Thread, Lock

from raspberry_pi.drivers.rp2040 import RP2040
from raspberry_pi.drivers.nano import NANO
from raspberry_pi.drivers.lidar import Lidar
from raspberry_pi.structures.state import Position
from raspberry_pi.perception import Perception
from raspberry_pi.planning import Planning


class Robot: 
    HOST = '172.20.10.3'
    MAIN_PORT = 5500
    MANUAL_PORT = 5501
    CAMERA_PORT = 5503

    BATTERY_CHECK_DELAY = 3
    WAITING_DELAY = 1

    class OdometryType(Enum):
        OFF = 0,
        ENCODERS_ONLY = 1,
        VISUAL_ONLY = 2
        FILTERED = 3

    class MappingType(Enum):
        OFF = 0,
        MAPPING = 1

    class ControlType(Enum):
        OFF = 0,
        MANUAL = 1,
        AUTONOMOUS = 2,
    
    # Devices
    nano = NANO()
    lidar = Lidar()

    # Settings and High Level states
    waiting = True
    odometry_type = OdometryType.OFF
    mapping_type = MappingType.OFF
    control_type = ControlType.OFF

    # Threads
    control_lock = Lock()
    battery_thread = None
    connection_thread = None

    # Low Level states
    actual_pos = None
    goal_pos = None

    # Socket
    manual_receiver = None
    odometry_transmitter = None
    

    @staticmethod
    def start():
        # start threads
        Robot.battery_thread = Thread(target=Robot.check_battery)
        Robot.battery_thread.start()

        Robot.connection_thread = Thread(target=Robot.connection_handler)
        Robot.connection_thread.start()

        while True:
            if Robot.waiting:
                time.sleep(Robot.WAITING_DELAY)
                continue
            
            # acquire command lock
            Robot.control_lock.acquire()

            ## LIDAR
            Robot.local_map = Robot.lidar.create_local_map()
            
            ## ODOMETRY
            if Robot.odometry_type == Robot.OdometryType.ENCODERS_ONLY: # only encoders odometry
                Robot.actual_pos = RP2040.get_position()

            elif Robot.odometry_type == Robot.OdometryType.VISUAL_ONLY: # only visual odometry
                Robot.actual_pos = Perception.visual_odometry()

            elif Robot.odometry_type == Robot.OdometryType.FILTERED:    # position filtering
                encoder_pos = RP2040.request_odometry()
                visual_pos = Perception.visual_odometry()
                Robot.actual_pos = Perception.position_filter(visual_pos, encoder_pos) # merge the positions
                RP2040.set_position(Robot.actual_pos) # update the encoders position to meet the merged position

            ## MAPPING
            if Robot.mapping_type == Robot.MappingType.MAPPING:
                Robot.global_map = Perception.map_construction(Robot.actual_pos, Robot.local_map, Robot.global_map)


            ## PLANNING
            if Robot.control_type == Robot.ControlType.AUTONOMOUS:
                    if goal_pos == None:
                        goal_pos = Planning.choose_exploration_goal(Robot.global_map, Robot.actual_pos)

                    path = Planning.path_planner(Robot.global_map, Robot.local_map, Robot.actual_pos, goal_pos)
                    RP2040.follow_path(path)

            elif Robot.control_type == Robot.ControlType.MANUAL:
                    vl, vth = manual_receiver.get_command() # TODO
                    vl, vth = Planning.obstacle_avoidance(vl, vth, Robot.local_map)
                    RP2040.set_target_velocity(vl, vth)
            
            # release command lock
            Robot.control_lock.release()

            # delay
            
            

    @staticmethod
    def check_battery():
        while True:
            # check battery # TODO
            time.sleep(Robot.BATTERY_CHECK_DELAY)

    @staticmethod
    def connection_handler():
        # start server
        connection = None
        connection_addr = None

        main_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        main_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        main_socket.bind((Robot.HOST, Robot.MAIN_PORT))
        main_socket.setblocking(False)
        main_socket.listen(1)

        while True:
            # not connected
            if connection == None:  
                try:
                    connection, connection_addr = main_socket.accept()
                    connection.setblocking(False)
                except BlockingIOError:
                    continue
            
            # connected
            else:       
                try:
                    received_data = connection.recv(32)
                except BlockingIOError:
                    continue

                if received_data == b'':
                    continue
                
                # command lock
                with Robot.control_lock:
                    # handle all commands
                    commands = received_data.decode().split('\n')[:-1]
                    for c in commands:
                        c = c.strip().split(' ')

                        ## COMMANDS HANDLING
        





if __name__ == "__main__":
    Robot.start()