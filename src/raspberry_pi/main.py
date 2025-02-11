import threading
import time
import socket
import sys
from robot.robot import Robot
from network.server import CommandServer
from utils.logger import get_logger

logger = get_logger(__name__)

# Global configuration
HOST = '192.168.1.103'
MAIN_PORT = 5500
MANUAL_PORT = 5501
DATA_PORT = 5502
BATTERY_CHECK_DELAY = 3
BATTERY_MIN_MV = 10500


def main():
    # 1. Initialize the Robot
    robot = Robot()
    logger.info("Robot initialized.")

    # 2. Start the command server
    command_server = CommandServer(robot, HOST, MAIN_PORT)
    command_server.start()
    logger.info(f"Command server started on {HOST}:{MAIN_PORT}")

    # 3. Battery monitoring 
    try:
        while True:
            battery_mv = robot.get_battery()
            logger.info(f"Battery voltage: {battery_mv} mV")
            if battery_mv < BATTERY_MIN_MV:
                logger.error("Battery too low. Initiating shutdown.")
                robot.stop()
                command_server.stop()
                sys.exit(1)
            time.sleep(BATTERY_CHECK_DELAY)
            
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