import socket
import threading
import time
from typing import Tuple

class ClientConnection:
    """
    Handles low-level socket communication with the robot.
    This class encapsulates connection, disconnection, and sending commands.
    """
    def __init__(self, robot_address: str, port: int):
        self.robot_address = robot_address
        self.port = port
        self.connection = None

    def connect(self) -> bool:
        """Establishes a connection to the robot."""
        self.connection = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.connection.settimeout(3)  # Set an explicit timeout
        try:
            self.connection.connect((self.robot_address, self.port))
        except (socket.timeout, ConnectionRefusedError, OSError) as e:
            print(f"Connection failed: {e}")
            self.connection = None
            return False
        return True

    def disconnect(self):
        """Closes the connection if it exists."""
        if self.connection:
            self.connection.close()
            self.connection = None

    def ping(self):
        """ Ping
        Returns:
            ok: bool
            time_ms: int
            battery_V: float
            control_type: str

    """
        ping_time_ms: int = None
        battery_V: float = None
        control_type: str = None
        map_name: str = None
        try:
            t = time.time()
            self.connection.send("SYS PNG\n".encode())
            response = self.connection.recv(32)
            ping_time_ms = int((time.time() - t)*1000)
            split = response.decode().strip().split()
            if split[0] == "OK":
                battery_V = float(split[1])/1000
                control_type = None if split[2] == "-" else split[2]
                map_name = None if split[3] == "-" else split[3]
                return True, ping_time_ms, battery_V, control_type, map_name
            return False, None , None, None, None
        except Exception as e:
            print(f"Ping failed: {e}")
            return False, None , None, None, None

    def start_manual_control(self) -> Tuple[bool, int]:
        """Sends a command to the robot to start manual control."""
        if not self.connection:
            return False, 0
        try:
            self.connection.send("CTL MAN\n".encode())
            response = self.connection.recv(32).decode().strip()
            split = response.split(" ")
            if split[0] == "OK" and int(split[1]):
                return True, int(split[1])
        except Exception as e:
            print(f"Manual control failed: {e}")
        return False, 0

    def stop_control(self) -> None:
        """Sends a command to the robot to stop manual control."""
        if not self.connection:
            return
        
        try:
            self.connection.send("CTL STP\n".encode())
        except Exception as e:
            print(f"Failed to stop control: {e}")


class DataReceiver:
    """
    A simple UDP server that listens on a specific port (e.g., 5502)
    for display data and uses an update callback to notify the view.
    """
    def __init__(self, udp_port: int, update_callback, buffer_size: int = 1024):
        self.udp_port = udp_port
        self.update_callback = update_callback  # Function to update the view with new data
        self.buffer_size = buffer_size
        self.sock = None
        self.thread = None
        self.running = False

    def start(self):
        """Starts the UDP server and spawns the worker thread."""
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("", self.udp_port))
        self.running = True
        self.thread = threading.Thread(target=self.worker, daemon=True)
        self.thread.start()

    def worker(self):
        """Worker thread that listens for UDP packets and processes them."""
        while self.running:
            try:
                # If the socket is already closed, exit the loop
                if self.sock.fileno() == -1:
                    print("Socket is closed, stopping worker")
                    break
                data = self.sock.recv(self.buffer_size)
                data = data.decode().strip()
                ok, size, global_map, lidar_points, robot_pose = self.parse_data(data)
                if ok:
                    self.update_callback(size, global_map, lidar_points, robot_pose)
            except Exception as e:
                print("Error in display server worker:", e)
                # Instead of calling stop() from within the worker, just break out of the loop.
                break
            time.sleep(0.1)
        # Cleanup after leaving the worker loop
        self.running = False
        if self.sock:
            self.sock.close()

    def stop(self):
        """Stops the UDP server and cleans up resources."""
        self.running = False
        if self.sock:
            self.sock.close()
        if self.thread and threading.current_thread() != self.thread:
            self.thread.join()
    
    def parse_data(self, data):
        """Parses the incoming data and returns the components."""
        lines = data.split("\n")
        print(lines)
        try:
            assert lines[0] == "DATA"
            size = int(lines[1])
            assert lines[3] == "GLOBAL_MAP"
            global_map = None
            if lines[4] != "-":
                global_map = [list(map(int, row.split())) for row in lines[4].split(";")]
            assert lines[6] == "LOCAL_MAP"
            lidar_points = None
            if lines[7] != "-":
                lidar_points = [tuple(map(float, point.split())) for point in lines[7].split(";")]
            assert lines[9] == "POSITION"
            robot_pos = None
            if lines[10] != "-":
                robot_pos = tuple(map(float, lines[10].split()))
            return True, size, global_map, lidar_points, robot_pos
        except AssertionError:
            return False, 0, None, None, None
        except Exception as e:
            return False, 0, None, None, None

MANUAL_INTERVAL = 0.1
class ManualTransmitter:
    """
    Handles a dedicated connection for manual control.
    This class connects to a given port, spawns a worker thread to handle communication,
    and uses an update callback to notify the application of received data.
    """
    def __init__(self, robot_address: str, port: int, input_handler):
        self.robot_address = robot_address
        self.port = port
        self.input_handler = input_handler 
        self.connection = None
        self.thread = None
        self.running = False

    def start(self):
        """Starts the manual server connection and worker thread."""
        self.connection = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.connection.settimeout(1)
        try:
            self.connection.connect((self.robot_address, self.port))
        except Exception as e:
            print("Error connecting to manual server:", e)
            return
        self.running = True
        self.thread = threading.Thread(target=self.worker, daemon=True)
        self.thread.start()

    def stop(self):
        """Stops the manual server and cleans up resources."""
        self.running = False
        if self.connection:
            self.connection.close()
        if self.thread:
            self.thread.join()

    def worker(self):
        """Worker thread that continuously reads from the manual connection."""
        while self.running:
            try:
                buffer = self.input_handler.get_buffer()
                data = f"{buffer['x']:.2f} {buffer['y']:.2f}\n"
                self.connection.send(data.encode())
            except Exception as e:
                print("Error in manual server worker:", e)
                self.connection.close()
                self.connection = None
                self.running = False
                break
            time.sleep(MANUAL_INTERVAL)

