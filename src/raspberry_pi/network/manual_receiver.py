import socket
import threading

class ManualReceiver:
    """
    TCP
    """
    _socket = None
    _connection = None
    _thread = None
    _lock = None
    _command = None

    @staticmethod
    def start(host, port): # TODO
        # socket
        ManualReceiver._socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        ManualReceiver._socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        ManualReceiver._socket.bind((host, port))
        ManualReceiver._socket.setblocking(False)
        ManualReceiver._socket.listen(1)

        # thread
        ManualReceiver._thread = threading.Thread(target=ManualReceiver._receiver_thread)
        ManualReceiver._lock = threading.Lock()


    
    @staticmethod
    def has_connected():
        with ManualReceiver._lock:
            return ManualReceiver._connection != None
        

    @staticmethod
    def stop(): # TODO
        with ManualReceiver._lock:
            if ManualReceiver._connection != None:
                ManualReceiver._connection.close()
                ManualReceiver._connection = None
            

        pass

    @staticmethod
    def get_command(): # TODO
        with ManualReceiver._lock:
            if ManualReceiver._socket == None or ManualReceiver._connection == None:
                return None
            


    @staticmethod
    def _receiver_thread(): # TODO
        while ManualReceiver._socket != None:
            with ManualReceiver._lock:
                if ManualReceiver._connection == None:
                    # check connection
                else:
                    # check command
            time.sleep()

        
