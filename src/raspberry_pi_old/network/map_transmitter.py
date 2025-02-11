import socket
import threading
import time 

# 1hz
class MapTransmitter:
    
    """
    TCP
    data type:
        "xy wz" where x, y, w, z are [0, 1] integers
            x: linear direction (0: backward, 1: forward)
            y: linear activation (0: off, 1: on)
            w: angular direction (0: right, 1: left)
            z: angular activation (0: off, 1: on)
    states:
        OFF: no thread, no socket
        ON: thread running, socket listening
        CONNECTED: thread running, socket listening, connection established
    """
    """ send all the same format but with optional values, the receiver prints what it has"""
    
    
    
    _size = 2000 # mm
    _resolution = 100 # mm
    _pixel_size = _size / _resolution 

    _robot = None
    _socket = None
    _connection = None
    _thread = None
    _lock = threading.Lock()


    @staticmethod
    def start(host: str, port: int, robot):
        MapTransmitter._robot = robot

        # socket
        MapTransmitter._socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        MapTransmitter._socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        MapTransmitter._socket.bind((host, port))
        MapTransmitter._socket.setblocking(False)
        MapTransmitter._socket.listen(1)

        # thread
        MapTransmitter._thread = threading.Thread(target=MapTransmitter._sender_thread)
        MapTransmitter._thread.start()

    
    @staticmethod
    def is_connected():
        with MapTransmitter._lock:
            return MapTransmitter._connection != None
        

    @staticmethod
    def stop():
        with MapTransmitter._lock:
            if MapTransmitter._connection != None:
                MapTransmitter._connection.close()
                MapTransmitter._connection = None
            MapTransmitter._socket.close()
            MapTransmitter._socket = None
        MapTransmitter._thread.join()
        MapTransmitter._thread = None
        MapTransmitter._robot = None



    @staticmethod
    def _sender_thread(): # TODO
        """
        "
        DATA\n
        <map_pixel_size>\n
        GLOBAL_MAP\n
        <global map row by row (- for empty) >\n
        LOCAL_MAP\n
        <list of lidar map points (- for empty) >\n
        POSITION\n
        <x> <y> <theta>\n
        "   
        - nxn is tha size
        - position is based on the global map (global coordinates)
            if no global map -> position is (0 0 0)
            needed for the orientation and the legend
        - local map is the ist of points from the lidar
            in frame coordinates
        - global map is a nxn occupancy grid
        """
        while True:
            if MapTransmitter._robot.__control_type != MapTransmitter._robot.ControlType.OFF:
                # read global map
                # read local map
                # read position
                pass
        
    @staticmethod
    def calculate_velocities():
        x, y = ManualReceiver._command
        if ManualReceiver._type == "keyboard":
            v_lin = x * ManualReceiver.LIN_SPEED
            v_ang = y * ManualReceiver.ANG_SPEED
            return v_lin, v_ang
        
        elif ManualReceiver._type == "joypad":
            if abs(x) < 0.25:
                x = 0
            if abs(y) < 0.1:
                y = 0
            v_lin = x * ManualReceiver.LIN_SPEED
            v_ang = y * ManualReceiver.ANG_SPEED
            return v_lin, v_ang
 