"""
    Robot Main Class

    INPUTS

    ### MAIN SOCKET CONNECTION ###
    TCP Socket
    all commands end with '\n'

    command types:
    - SYSTEM:
        - ping     
            "SYS PNG" -> "OK <battery_mv> <control_type> <map_name>"
            battery_mv: battery voltage in millivolts
            control_type: 0 off, 1 manual, 2 autonomous
            map_name: name of the global map (- if no map)

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
                "CTL MAN" -> "OK <manual_port>"
                starts manual control thread and server
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
            stops the robot (lidar, motors ecc)
            

    - MAPPING:
        - new_map
            "MAP NEW <name>" -> "OK"
            creates a new map with the given name
            "KO" if map already exists

        - discard
            "MAP DIS" -> "OK"
            discards the global map
            ensures:
                in mapping mode => stops mapping

        - save_map
            "MAP SAV" -> "OK"
            saves the global map to the given file
            "KO" if no global map
        
        - load_map
            "MAP LOD <name>" -> "OK"
            sets global map to the given map
        
        - start_mapping
            "MAP STR" -> "OK"
            starts mapping
            "KO" if already in mapping mode

        - finish_mapping
            "MAP STP" -> "OK" 
            finishes mapping and keeps the map as global map
            "KO" if not in mapping mode


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
from threading import Thread, Lock, Event

from raspberry_pi.robot import Robot
from raspberry_pi.utils import timing_decorator
from raspberry_pi.network.manual_receiver import ManualReceiver

HOST = '192.168.1.103'
MAIN_PORT = 5500
MANUAL_PORT = 5501
CAMERA_PORT = 5503
# delays
WAITING_DELAY = 1
SERVER_DELAY = 0.2
# battery check
BATTERY_CHECK_DELAY = 3
BATTERY_MIN_MV = 10500

robot = None

battery_thread = None
stop_battery = Event()

main_socket = None
connection = None
    
@timing_decorator
def main():
    global robot, battery_thread, main_socket, connection
    robot = Robot()

    # SETUP BATTERY CHECK
    battery_thread = Thread(target=battery_check_worker, args=(robot,), daemon=True)
    battery_thread.start()

    # SETUP SERVER
    main_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    main_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    main_socket.bind((HOST, MAIN_PORT))
    main_socket.setblocking(False)
    main_socket.listen(1)

    # MAIN LOOP
    while True:
        if connection == None:
            # check connection
            try:
                connection, _ = main_socket.accept()
                connection.setblocking(False)
                # robot.start()
            except BlockingIOError:
                time.sleep(WAITING_DELAY)
                continue
        else:
            # receive data
            try:
                received_data = connection.recv(32)
            except (ConnectionResetError, BrokenPipeError): 
                close_connection()
                time.sleep(WAITING_DELAY)
                continue
            except BlockingIOError:
                time.sleep(SERVER_DELAY)
                continue
            if received_data == b'':
                time.sleep(SERVER_DELAY)
                continue

            # HANDLE COMMANDS
            commands = received_data.decode().split('\n')
            print(commands)
            for c in commands:
                c = c.strip().split(' ')
                if c[0] == "SYS":
                    if c[1] == "PNG":
                        """Ping
                            "SYS PNG" -> "OK <battery_mv> <control_type> <map_name>"
                                battery_mv: battery voltage in millivolts
                                control_type: 0 off, 1 manual, 2 autonomous
                                map_name: name of the global map (- if no map) """
                        battery = str(robot.get_battery())
                        control = str(robot.get_control_type())
                        map_name = '-'
                        res = f"OK {battery} {control} {map_name}\n"
                        connection.send(res.encode())

                elif c[0] == "MAP":

                    if c[1] == "NEW":
                        """New Map
                            "MAP NEW <name>" -> "OK"
                            creates a new map with the given name
                            "KO" if alraedy has a map
                        """
                        try:
                            robot.new_map(c[2])
                            res = "OK\n"
                        except:
                            res = "KO\n"
                        connection.send(res.encode())

                    elif c[1] == "DIS":
                        """ Discard Map
                            "MAP DIS" -> "OK"
                            discards the global map (does not eliminate the map file)
                        """
                        try:
                            robot.discard_map()
                            res = "OK\n"
                        except: # no map to discard
                            res = "KO\n"
                        connection.send(res.encode())
            
                    elif c[1] == "SAV":
                        """ Save Map
                            "MAP SAV" -> "OK"
                        """
                        try:
                            robot.save_map()
                            res = "OK\n"
                        except:
                            res = "KO\n"
                        connection.send(res.encode())
                        
                    elif c[1] == "LOD":
                        # Load Map
                        # "MAP LOD <name>" -> "OK"
                        pass

                    elif c[1] == "STR":
                        """ Start Mapping
                            "MAP STR" -> "OK"
                        """
                        # Start Mapping
                        # "MAP STR" -> "OK"
                        try:
                            robot.start_mapping()
                            res = "OK"
                        except:
                            res = "KO"
                        connection.send(res.encode())

                    elif c[1] == "STP":
                        # Stop Mapping
                        # "MAP STP" -> "OK"
                        try:
                            robot.stop_mapping()
                            res = "OK"
                        except:
                            res = "KO"
                        connection.send(res.encode())

                elif c[0] == "CTL":
                    if c[1] == "STP":
                        """ Stop Control
                            "CTL STP" -> "OK"
                            completely stops the robot (lidar ecc)
                        """
                        if ManualReceiver.is_connected():
                            ManualReceiver.stop()
                        robot.stop()
                        connection.send("OK\n".encode())

                    elif c[1] == "MAN":
                        """ Start Manual Control
                            "CTL MAN" -> "OK"
                            starts manual receiver
                        """
                        robot.start()
                        ManualReceiver.start(HOST, MANUAL_PORT, robot)
                        connection.send(f"OK {MANUAL_PORT}\n".encode())
            

def close_connection():
    robot.stop()
    if connection != None:
        connection.close()
        connection = None

def close_all():
    stop_battery.set()
    battery_thread.join()
    battery_thread = None
    close_connection()
    main_socket.close()
    main_socket = None

def shutdown():
    close_all()
    # shutdown system
    pass

def battery_check_worker(robot):
    while not stop_battery.is_set():
        battery_mv = robot.get_battery()
        if battery_mv < BATTERY_MIN_MV:
            robot.stop()
            shutdown()
            return
        time.sleep(BATTERY_CHECK_DELAY)



# class Main: 
    

#     class Status(Enum):
#         DISCONNECTED = 0,
#         IDLE = 1,
#         ACTIVE = 2,
    
    
#     # Settings and High Level states
#     status = Status.DISCONNECTED
#     #
#     battery_thread = None
#     connection_thread = None

#     # Socket
#     main_socket = None
#     main_connection = None
#     manual_receiver = None
#     odometry_transmitter = None

    

#     @staticmethod
#     def start():
#         # start drivers
        

#         # setup main socket
#         Robot.main_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
#         Robot.main_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
#         Robot.main_socket.bind((Robot.HOST, Robot.MAIN_PORT))
#         Robot.main_socket.setblocking(False)
#         Robot.main_socket.listen(1)
        
#         # control loop
#         while True:
#             # check connection??

#             # DISCONNECTED
#             if Robot.status == Robot.Status.DISCONNECTED:
#                 # check connection
#                 try:
#                     Robot.main_connection, _ = Robot.main_socket.accept()
#                     Robot.main_connection.setblocking(False)
#                     Robot.status = Robot.Status.IDLE
#                     Robot.connection_thread = Thread(target=Robot.connection_handler, daemon=True)
#                     Robot.connection_thread.start()
#                 except BlockingIOError:
#                     Robot.main_connection = None
#                     time.sleep(Robot.WAITING_DELAY)
#                     continue
            
#             # IDLE
#             elif Robot.status == Robot.Status.IDLE:
#                 time.sleep(Robot.WAITING_DELAY)
#                 continue
            
#             # ACTIVE
#             elif Robot.status == Robot.Status.ACTIVE:
#                 with Robot.control_lock:
#                     ## LIDAR
#                     Robot.local_map = Lidar.produce_local_map()
                    
#                     ## ODOMETRY
#                     if Robot.odometry_type == Robot.OdometryType.ENCODERS_ONLY: # only encoders odometry
#                         Robot.actual_pos = RP2040.get_position()
#                     elif Robot.odometry_type == Robot.OdometryType.FILTERED:    # position filtering
#                         encoder_pos = RP2040.request_odometry()
#                         visual_pos = Perception.visual_odometry()
#                         Robot.actual_pos = Perception.position_filter(visual_pos, encoder_pos) # merge the positions
#                         RP2040.set_position(Robot.actual_pos) # update the encoders position to meet the merged position

#                     ## MAPPING
#                     if Robot.mapping_type == Robot.MappingType.MAPPING:
#                         Robot.global_map = Perception.map_construction(Robot.actual_pos, Robot.local_map, Robot.global_map)

#                     ## PLANNING
#                     if Robot.control_type == Robot.ControlType.AUTONOMOUS:
#                             if goal_pos == None:
#                                 goal_pos = Planning.choose_exploration_goal(Robot.global_map, Robot.actual_pos)
#                             path = Planning.path_planner(Robot.global_map, Robot.local_map, Robot.actual_pos, goal_pos)
#                             RP2040.follow_path(path)
#                     elif Robot.control_type == Robot.ControlType.MANUAL:
#                             vl, vth = ManualReceiver.get_command() # TODO
#                             vl, vth = Planning.obstacle_avoidance(vl, vth, Robot.local_map)
#                             RP2040.set_target_velocity(vl, vth)
#                     elif Robot.control_type == Robot.ControlType.OFF:
#                         RP2040.stop_motors()
                
#                 time.sleep(0.1)

#     @staticmethod
#     def stop():
#         Robot.status = Robot.Status.DISCONNECTED
#         # stop threads
#         Robot.main_connection.close()
#         Robot.main_connection = None
#         pass

#     @staticmethod
#     def check_battery():
#         while True:
#             battery_mv = NANO.get_battery()
#             if battery_mv < Robot.BATTERY_MIN_MV:
#                 Robot.stop()
#                 Robot.shutdown()
#                 return
#             time.sleep(Robot.BATTERY_CHECK_DELAY)

#     @staticmethod
#     def connection_handler():
#         """THREAD
#         Handles the main connection
#         """
#         while Robot.status != Robot.Status.DISCONNECTED:
#             try:
#                 # receive data
#                 received_data = Robot.main_connection.recv(32)

#             except (ConnectionResetError, BrokenPipeError): 
#                 # lost connection
#                 Robot.status = Robot.Status.DISCONNECTED
#                 Robot.main_connection = None
#                 return
#             except BlockingIOError: # no data
#                 continue
#             if received_data == b'': # no data
#                 continue

#             # divide commands
#             commands = received_data.decode().split('\n')
#             for c in commands:
#                 # handle single command
#                 c = c.strip().split(' ')

#                 if c[0] == "SYS":
#                     if c[1] == "PNG":
#                         # Ping
#                         # "SYS PNG" -> "OK" 
#                         pass

#                     # elif c[1] == "BAT":
#                     #     # Battery
#                     #     # "SYS BAT" -> "OK <battery_mv>"
#                     #     pass

#                     elif c[1] == "STM":
#                         # Stop Motors
#                         # "SYS STM" -> "OK"
#                         Robot.control_type = Robot.ControlType.OFF
#                         with Robot.control_lock:
#                             RP2040.stop_motors()

#                     elif c[1] == "STP":
#                         # Stop All
#                         # "SYS STP" -> "OK"
#                         Robot.stop()
#                         return
    
#                     elif c[1] == "SHD":
#                         # Shutdown
#                         # "SYS SHD" -> "OK"
#                         # TODO
#                         pass

#                 elif c[0] == "CTL":
#                     # manual control
#                     if c[1] == "MAN":
#                         if c[2] == "STR":
#                             # Start Manual Control
#                             # "CTL MAN STR" -> "OK <manual_port>"
#                             # TODO
#                             pass
#                         elif c[2] == "STP":
#                             # Stop Manual Control
#                             # "CTL MAN STP" -> "OK <manual_port>"
#                             # TODO
#                             pass
#                     elif c[1] == "AUT":
#                         if c[2] == "STR":
#                             # Start Autonomous Control
#                             # "CTL AUT STR" -> "OK"
#                             # TODO
#                             pass
#                         elif c[2] == "STP":
#                             # Stop Autonomous Control
#                             # "CTL AUT STP" -> "OK"
#                             # TODO
#                             pass
#                         elif c[2] == "SET":
#                             # Set Goal
#                             # "CTL AUT SET <x> <y>" -> "OK"
#                             # TODO
#                             pass



                
#                 # command lock
#                 with Robot.control_lock:
#                     # handle all commands
#                     for c in commands:
#                         c = c.strip().split(' ')

#                         ## COMMANDS HANDLING


#     @staticmethod    
#     def ping() -> str:
#         # battery
#         # rp2040 nano ping
#         # lidar status
#         pass




if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        if robot:
            robot.stop()
            del robot