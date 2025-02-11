import socket
import threading
import time
from raspberry_pi.utils.logger import get_logger
from raspberry_pi.config import COMMAND_SERVER_CONFIG, MANUAL_SERVER_CONFIG

logger = get_logger(__name__)

class ManualReceiver:
    def __init__(self, robot):
        self.robot = robot  # The robot instance to send manual commands to.
        self._running = False
        self._thread = None
        self._server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._server_socket.bind((COMMAND_SERVER_CONFIG.HOST, MANUAL_SERVER_CONFIG.PORT))
        self._server_socket.listen(1)
        self._connection = None

    def start(self):
        self._running = True
        self._thread = threading.Thread(target=self._listen_for_connections, daemon=True)
        self._thread.start()
        logger.info(f"ManualReceiver started on {self.host}:{self.port}")
    
    def stop(self):
        self._running = False
        if self._connection:
            self._connection.close()
        self._server_socket.close()

    def _listen_for_connections(self):
        while self._running:
            try:
                if self._connection is None:
                    self._connection, client_address = self._server_socket.accept()
                    logger.info(f"ManualReceiver: Accepted connection from {client_address}")
                    # Handle client
                    self._handle_client(self._connection)
                    self._connection = None

            except Exception as e:
                logger.error(f"Error accepting connection: {e}")

    def _handle_client(self, connection: socket.socket):
        with connection:
            try:
                connection.settimeout(2)
                buffer = ""
                while True:
                    try:
                        data = connection.recv(1024).decode()
                        if not data:
                            continue
                        buffer += data
                        while "\n" in buffer:
                            command, buffer = buffer.split("\n", 1)
                            v_lin, v_ang = self._process_command(command)
                            self._robot.set_target_velocity(v_lin, v_ang)
                    
                    except socket.timeout:
                        continue

            except Exception as e:
                logger.error(f"Error handling client: {e}")

    def _process_command(self, command: str) -> str:
        x, y = map(float, command.strip().split(" "))

        x = 0 if abs(x) < 0.25 else x
        y = 0 if abs(y) < 0.25 else y

        v_lin = x * MANUAL_SERVER_CONFIG.LIN_VEL
        v_ang = y * MANUAL_SERVER_CONFIG.ANG_VEL
        return v_lin, v_ang

        
