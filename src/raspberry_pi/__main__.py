import threading
import time
import socket
import sys
from raspberry_pi.robot.robot import Robot
from raspberry_pi.network.server import CommandServer
from raspberry_pi.config import COMMAND_SERVER_CONFIG as server_config, ROBOT_CONFIG as robot_config
from raspberry_pi.utils.logger import get_logger

logger = get_logger(__name__)

# Global configuration
# HOST = '192.168.1.103'
# MAIN_PORT = 5500
# MANUAL_PORT = 5501
# DATA_PORT = 5502
# BATTERY_CHECK_DELAY = 3
# BATTERY_MIN_MV = 10500


def main():
    # 1. Initialize the Robot
    robot = Robot()
    logger.info("Robot initialized.")

    # 2. Start the command server
    command_server = CommandServer(robot)
    command_server.start()
    logger.info(f"Command server started on {server_config.HOST}:{server_config.PORT}")

    # 3. Battery monitoring 
    try:
        while True:
            battery_mv = robot.get_battery()
            logger.info(f"Battery voltage: {battery_mv} mV")
            if battery_mv < robot_config.BATTERY_MIN_MV:
                logger.error("Battery too low. Initiating shutdown.")
                robot.stop()
                command_server.stop()
                sys.exit(1)
            time.sleep(robot_config.BATTERY_CHECK_INTERVAL)
            
    except KeyboardInterrupt:
        logger.info("Keyboard interrupt detected. Shutting down...")
        robot.stop()
        command_server.stop()
        sys.exit(0)
        
    except Exception as e:
        logger.error(f"An error occurred: {e}")
        robot.stop()
        command_server.stop()
        sys.exit(1)

if __name__ == "__main__":
    main()