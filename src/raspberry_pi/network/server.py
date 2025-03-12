import socket
import threading
import time
from typing import List

from raspberry_pi.network.data_transmitter import DataTransmitter
from raspberry_pi.network.manual_receiver import ManualReceiver
from raspberry_pi.utils.logger import get_logger
from raspberry_pi.config import COMMAND_SERVER_CONFIG, MANUAL_SERVER_CONFIG

logger = get_logger(__name__)

class CommandServer:
    def __init__(self, robot):
        self.robot = robot
        self._server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._server_socket.bind((COMMAND_SERVER_CONFIG.HOST, COMMAND_SERVER_CONFIG.PORT))
        self._server_socket.listen(1)
        self._running = False
        self._connection = None
        self._thread = threading.Thread(target=self._listen_for_connections, daemon=True)
        self._manual_receiver = None
        self._data_transmitter = None
    
    def start(self):
        self._running = True
        self._thread.start()
        logger.info("CommandServer thread started.")

    def stop(self):
        self._running = False
        if self._connection:
            self._connection.close()
        if self._thread.is_alive():
            self._thread.join()
        self._server_socket.close()

        # if self._manual_receiver:
        #     self._manual_receiver.stop()
        #     self._manual_receiver = None
        # if self._data_transmitter:
        #     self._data_transmitter.stop()
        #     self._data_transmitter = None
        
    
    def _listen_for_connections(self):
        while self._running:
            try:
                if self._connection is None:
                    self._connection, client_address = self._server_socket.accept()
                    logger.info(f"Accepted connection from {client_address}")
                    # Start DataTransmitter
                    if self._data_transmitter is None:
                        self._data_transmitter = DataTransmitter(client_address[0], self.robot)
                        self._data_transmitter.start()

                    # Handle client
                    self._handle_client(self._connection)

                    # Stop DataTransmitter
                    if self._data_transmitter:
                        self._data_transmitter.stop()
                        self._data_transmitter = None
                    # Stop ManualReceiver
                    if self._manual_receiver:
                        self._manual_receiver.stop()
                        self._manual_receiver = None
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
                            logger.info("Client disconnected.")
                            break
                        
                        buffer += data
                        while "\n" in buffer:
                            command, buffer = buffer.split("\n", 1)
                            logger.info("Received command: " + command)
                            response = self._process_command(command.strip())
                            connection.send((response + "\n").encode())

                    except socket.timeout:
                        continue
                    except (socket.error, ConnectionResetError) as e:
                        logger.warning(f"Client connection error: {e}")
                        break  

            except (ConnectionResetError, BrokenPipeError):
                logger.info("Client forcefully disconnected.")
            except Exception as e:
                logger.error(f"Error handling client: {e}")

    def _process_command(self, command: str) -> None:
        logger.info(f"Received command: {command}")
        tokens = command.split()
        command_type = tokens.pop(0)
        try:
            if command_type == "SYS":
                return self._process_sys_command(tokens)
            elif command_type == "MAP":
                return self._process_map_command(tokens)
            elif command_type == "ODO":
                return self._process_odometry_command(tokens)
            elif command_type == "CTL":
                return self._process_control_command(tokens)
            return "KO"
            
        except Exception as e:
            logger.error(f"Error processing command: {e}", e)
            return "KO"
    
    def _process_sys_command(self, tokens: List[str]) -> str:
        command = tokens.pop(0)
        if command == "PNG":
            """Ping
                "SYS PNG" -> "OK <battery_mv> <control_type> <map_name>"
                    battery_mv: battery voltage in millivolts
                    control_type: 0 off, 1 manual, 2 autonomous
                    map_name: name of the global map (- if no map) """
            battery = str(self.robot.get_battery())
            control = str(self.robot.get_control_type())
            map_name = '-'
            return f"OK {battery} {control} {map_name}"
        return "KO"

    def _process_map_command(self, tokens: List[str]) -> str:
        command = tokens.pop(0)
        if command == "NEW":
            """New Map
                "MAP NEW <name>" -> "OK"
                creates a new map with the given name
                "KO" if alraedy has a map
            """
            map_name = tokens.pop(0)
            self.robot.new_global_map(map_name)
            return "OK"

        elif command == "DIS":
            """ Discard Map
                "MAP DIS" -> "OK"
                discards the global map (does not eliminate the map file)
            """
            self.robot.discard_global_map()
            return "OK"

        elif command == "SAV":
            """ Save Map
                "MAP SAV" -> "OK"
            """
            self.robot.save_global_map()
            return "OK"
            
        elif command == "LOD":
            # Load Map
            # "MAP LOD <name>" -> "OK"
            pass

        elif command == "STR":
            """ Start Mapping
                "MAP STR" -> "OK"
            """
            self.robot.start_mapping()
            return "OK"

        elif command == "STP":
            """ Stop Mapping
                "MAP STP" -> "OK"
            """
            self.robot.stop_mapping()
            return "OK"
        return "KO"

    def _process_odometry_command(self, tokens: List[str]) -> str:
        command = tokens.pop(0)
        if command == "RST":
            """ Reset Odometry
                "ODO RST" -> "OK"
                resets the odometry of the robot
            """
            self.robot.reset_odometry()
            return "OK"

    def _process_control_command(self, tokens: List[str]) -> str:
        command = tokens.pop(0)
        if command == "STP":
            """ Stop Control
                "CTL STP" -> "OK"
                completely stops the robot (lidar ecc)
            """
            if self._manual_receiver is not None:
                self._manual_receiver.stop()
                self._manual_receiver = None
            self.robot.set_control('off')
            return "OK"

        elif command == "MAN":
            """ Start Manual Control
                "CTL MAN" -> "OK"
                starts manual receiver
            """
            if self._manual_receiver is None:
                self._manual_receiver = ManualReceiver(self.robot)
                self._manual_receiver.start()
            self.robot.set_control('manual')
            return f"OK {MANUAL_SERVER_CONFIG.PORT}" 
        return "KO"