"""
    Robot Main Class

    INPUTS

    ### MAIN SOCKET CONNECTION ###
    TCP Socket
    all commands end with '\n'

    command types:
    - SYSTEM:
        - ping [pure]      
            "SYS PNG" -> "OK"

        - battery [pure]
            "SYS BAT" -> "OK <battery_mv>"
            battery_mv: battery voltage in millivolts

        - stop_motors   
            "SYS STM" -> "OK"
            stops motors and resets control??
            ensures:
                - control_type = OFF 
                ...
        - stop          
            "SYS STP" -> "OK"
            stops connections and motors
            ensures:
                no running processes after calling

        - shutdown  
            "SYS SHD" -> "OK"
            closes connections and shutdowns the robot
            ensures:
                raspberry pi is shutdown

    - CONTROL:
        - manual 
            - start [thread]
                "CTL MAN STR" -> "OK <manual_port>"
                starts manual control thread and server
            - stop
                "CTL MAN STP" -> "OK"
                stops manual control thread and server
        - autonomous
            - start
                "CTL AUT STR" -> "OK"
                starts autonomous control
            - stop
                "CTL AUT STP" -> "OK"
                stops autonomous control
            - set_goal
                "CTL AUT SET <x> <y>" -> "OK"
                sets the goal position
                "KO" if not in autonomous control mode
        - stop
            "CTL STP" -> "OK"
            sets control type to OFF
            ensures:
                no power to motors
                odometry and mapping may still running

    - MAPPING:
        - start_mapping
            "MAP STR" -> "OK"
            starts mapping
            ensures:
                global map exists => it expands it
                alredy scanning => continues

        - finish_mapping
            "MAP FIN" -> "OK" 
            finishes mapping and keeps the map as global map
            "KO" if not in mapping mode

        - save_map
            "MAP SAV <filename??>" -> "OK"
            saves the global map to the given file
            ensures:
                in mapping mode => continues mapping
                not in mapping mode => stops mapping

        - discard
            "MAP DIS" -> "OK"
            discards the global map
            ensures:
                in mapping mode => stops mapping
        
        - use_map
            "MAP USE <filename>" -> "OK"
            sets global map to the given map

    - LOCALIZATION ??
        - set_mode 
            "LOC SET <mode>" -> "OK"
            mode:   0   off ?? remove and allow only if in autonomous mode??
                    1   encoders
                    2   visual
                    3   filtered    
    - DATA
        - environment 
            ?? do i need another thread/server, how do i devide the commands
            ?? maybe i use a thread and server only for the stream of data
            - start [thread]
                "DTA ENV STR <delay_ms>" -> "OK"
                starts environment data thread
                sends data with the given delay
                data:
                    - odometry
                    - lidar scan
                    - subsection of global map
            - stop
                "DTA ENV STP" -> "OK"
                stops environment data thread

            - get_map ???
                "DTA ENV MAP" -> "OK <map>"
                returns the global map
            
        - camera [thread]
        - logs 
    - SETTINGS ("SET")
        ...
    

    high level functions:
        MAPPING
            AUTONOMOUS
            MANUAL

        LOCALIZE (in given map)
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

from raspberry_pi.network.manual_receiver import ManualReceiver


class Robot: 
    HOST = '172.20.10.3'
    MAIN_PORT = 5500
    MANUAL_PORT = 5501
    CAMERA_PORT = 5503

    # delays
    BATTERY_CHECK_DELAY = 3
    WAITING_DELAY = 1

    BATTERY_MIN_MV = 10500

    class Status(Enum):
        DISCONNECTED = 0,
        IDLE = 1,
        ACTIVE = 2,
    
    class OdometryType(Enum):
        OFF = 0,
        ENCODERS_ONLY = 1,
        FILTERED = 2

    class MappingType(Enum):
        OFF = 0,
        MAPPING = 1

    class ControlType(Enum):
        OFF = 0,
        MANUAL = 1,
        AUTONOMOUS = 2,
    
    # Settings and High Level states
    status = Status.DISCONNECTED
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
    local_map = None
    global_map = None

    # Socket
    main_socket = None
    main_connection = None
    manual_receiver = None
    odometry_transmitter = None

    

    @staticmethod
    def start():
        # start drivers
        RP2040.start()
        NANO.start()
        Lidar.start()
        
        # start threads
        Robot.battery_thread = Thread(target=Robot.check_battery, daemon=True)
        Robot.battery_thread.start()

        # setup main socket
        Robot.main_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        Robot.main_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        Robot.main_socket.bind((Robot.HOST, Robot.MAIN_PORT))
        Robot.main_socket.setblocking(False)
        Robot.main_socket.listen(1)
        
        # control loop
        while True:
            # check connection??

            # DISCONNECTED
            if Robot.status == Robot.Status.DISCONNECTED:
                # check connection
                try:
                    Robot.main_connection, _ = Robot.main_socket.accept()
                    Robot.main_connection.setblocking(False)
                    Robot.status = Robot.Status.IDLE
                    Robot.connection_thread = Thread(target=Robot.connection_handler, daemon=True)
                    Robot.connection_thread.start()
                except BlockingIOError:
                    Robot.main_connection = None
                    time.sleep(Robot.WAITING_DELAY)
                    continue
            
            # IDLE
            elif Robot.status == Robot.Status.IDLE:
                time.sleep(Robot.WAITING_DELAY)
                continue
            
            # ACTIVE
            elif Robot.status == Robot.Status.ACTIVE:
                with Robot.control_lock:
                    ## LIDAR
                    Robot.local_map = Lidar.produce_local_map()
                    
                    ## ODOMETRY
                    if Robot.odometry_type == Robot.OdometryType.ENCODERS_ONLY: # only encoders odometry
                        Robot.actual_pos = RP2040.get_position()
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
                            vl, vth = ManualReceiver.get_command() # TODO
                            vl, vth = Planning.obstacle_avoidance(vl, vth, Robot.local_map)
                            RP2040.set_target_velocity(vl, vth)
                    elif Robot.control_type == Robot.ControlType.OFF:
                        RP2040.stop_motors()
                
                time.sleep(0.1)

    def stop():
        # TODO
        pass

    @staticmethod
    def check_battery():
        while True:
            battery_mv = NANO.get_battery()
            if battery_mv < Robot.BATTERY_MIN_MV:
                Robot.stop()
                Robot.shutdown()
                return
            time.sleep(Robot.BATTERY_CHECK_DELAY)

    @staticmethod
    def connection_handler():
        """THREAD
        Handles the main connection
        """

        while Robot.status != Robot.Status.DISCONNECTED:
            try:
                # receive data
                received_data = Robot.main_connection.recv(32)

            except (ConnectionResetError, BrokenPipeError): 
                # lost connection
                Robot.status = Robot.Status.DISCONNECTED
                Robot.main_connection = None
                return
            except BlockingIOError: # no data
                continue
            if received_data == b'': # no data
                continue

            # divide commands
            commands = received_data.decode().split('\n')
            for c in commands:
                # handle single command
                c = c.strip().split(' ')

                if c[0] == "SYS":
                    if c[1] == "PNG":
                        # Ping
                        # "SYS PNG" -> "OK" 
                        pass

                    # elif c[1] == "BAT":
                    #     # Battery
                    #     # "SYS BAT" -> "OK <battery_mv>"
                    #     pass

                    elif c[1] == "STM":
                        # Stop Motors
                        # "SYS STM" -> "OK"
                        Robot.control_type = Robot.ControlType.OFF

                    elif c[1] == "STP":
                        # Stop All
                        # "SYS STP" -> "OK"
                        # TODO
                        pass

                    elif c[1] == "SHD":
                        # Shutdown
                        # "SYS SHD" -> "OK"
                        # TODO
                        pass

                elif c[0] == "CTL":
                    # manual control
                    if c[1] == "MAN":
                        if c[2] == "STR":
                            # Start Manual Control
                            # "CTL MAN STR" -> "OK <manual_port>"
                            # TODO
                            pass
                        elif c[2] == "STP":
                            # Stop Manual Control
                            # "CTL MAN STP" -> "OK <manual_port>"
                            # TODO
                            pass
                    



                
                # command lock
                with Robot.control_lock:
                    # handle all commands
                    for c in commands:
                        c = c.strip().split(' ')

                        ## COMMANDS HANDLING

    @staticmethod    
    def ping() -> str:
        # battery
        # rp2040 nano ping
        # lidar status
        pass




if __name__ == "__main__":
    Robot.start()