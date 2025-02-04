import socket
import threading
import time 

class ManualReceiver:
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
    LIN_SPEED = 270
    ANG_SPEED = 800
    MAX_TIME_WITHOUT_COMMAND = 1

    _robot = None
    _socket = None
    _connection = None
    _thread = None
    _lock = threading.Lock()
    _type = None
    _command = (0.0, 0.0)
    _last_received = None

    @staticmethod
    def start(host, port: int, robot):
        ManualReceiver._robot = robot

        # socket
        ManualReceiver._socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        ManualReceiver._socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        ManualReceiver._socket.bind((host, port))
        ManualReceiver._socket.setblocking(False)
        ManualReceiver._socket.listen(1)

        # thread
        ManualReceiver._thread = threading.Thread(target=ManualReceiver._receiver_thread)
        ManualReceiver._thread.start()

    
    @staticmethod
    def is_connected():
        with ManualReceiver._lock:
            return ManualReceiver._connection != None
        

    @staticmethod
    def stop():
        with ManualReceiver._lock:
            if ManualReceiver._connection != None:
                ManualReceiver._connection.close()
                ManualReceiver._connection = None
            ManualReceiver._socket.close()
            ManualReceiver._socket = None
        ManualReceiver._thread.join()
        ManualReceiver._thread = None
        ManualReceiver._command = (0, 0)
        ManualReceiver._last_received = None
        ManualReceiver._type = None
        ManualReceiver._robot = None

    @staticmethod
    def get_command(): # TODO
        with ManualReceiver._lock:
            if ManualReceiver._socket == None or ManualReceiver._connection == None:
                return None
            


    @staticmethod
    def _receiver_thread(): # TODO
        while True:
            with ManualReceiver._lock:
                if ManualReceiver._connection == None:
                    # check connection
                    try:
                        ManualReceiver._connection, _ = ManualReceiver._socket.accept()
                        ManualReceiver._connection.setblocking(False)
                    except BlockingIOError:
                        pass
                
                else:
                    # read dat
                    try:
                        data = ManualReceiver._connection.recv(25).decode().strip()
                        if data == "":
                            continue

                        split = data.split("\n")[0].split(" ")
                        if len(split) != 3:
                            print("Invalid man receiver: ", split, "len: ",  len(split))
                            continue
                        if split[0] == "KEY":
                            ManualReceiver._type = "keyboard"
                        elif split[0] == "JOY":
                            ManualReceiver._type = "joypad"
                        x = int(split[1])
                        y = int(split[2])
                        ManualReceiver._command = (x, y)
                        ManualReceiver._last_received = time.time()
                        v_lin, v_ang = ManualReceiver.calculate_velocities()
                        ManualReceiver._robot.set_target_velocity(v_lin, v_ang)

                    except BlockingIOError:
                        # inactive
                        if ManualReceiver._last_received != None and time.time() - ManualReceiver._last_received > ManualReceiver.MAX_TIME_WITHOUT_COMMAND:
                            ManualReceiver._connection.close()
                            ManualReceiver._connection = None
                            return

            time.sleep(0.1)

        
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
 