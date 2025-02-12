# config.py
"""
Global configuration file for the autonomous robot project.
This file defines network settings, device parameters, mapping options,
and other constants used throughout the project.
"""
from dataclasses import dataclass

@dataclass
class RobotConfig:
    CONTROL_LOOP_INTERVAL: float
    MAP_FOLDER: str
    SCANS_FOLDER: str
    BATTERY_MIN_MV: int
    BATTERY_CHECK_INTERVAL: float
    GLOBAL_MAP_SIZE_MM: int
    GLOBAL_MAP_RESOLUTION: int
ROBOT_CONFIG = RobotConfig(
    CONTROL_LOOP_INTERVAL = 0.1,
    MAP_FOLDER = "../data/maps",
    SCANS_FOLDER = "../data/scans",

    BATTERY_MIN_MV = 10500,
    BATTERY_CHECK_INTERVAL = 5,

    GLOBAL_MAP_SIZE_MM = 10000,
    GLOBAL_MAP_RESOLUTION = 100,
)

@dataclass
class CommandServerConfig:
    HOST: str
    PORT: int
COMMAND_SERVER_CONFIG = CommandServerConfig(
    HOST = "192.168.1.103",          
    PORT = 5500,       
)

@dataclass
class ManualServerConfig:
    PORT: int
    LIN_VEL: float
    ANG_VEL: float
MANUAL_SERVER_CONFIG = ManualServerConfig(        
    PORT = 5501,  
    LIN_VEL = 270,
    ANG_VEL = 800,
)

@dataclass
class DataServerConfig:
    INTERVAL: float
    PORT: int
    SIZE_MM: int
DATA_SERVER_CONFIG = DataServerConfig(
    PORT = 5502,
    INTERVAL = 0.5,
    SIZE_MM = 5000, # mm
)


@dataclass
class RP2040Config:
    PORT: str
    BAUD_RATE: int
    TIMEOUT: float
RP2040_CONFIG = RP2040Config(
    PORT="/dev/ttyAMA0",
    BAUD_RATE=115200,
    TIMEOUT= 1,
)

@dataclass
class NANOConfig:
    BAUD_RATE: int
    PORT: str
    TIMEOUT: float
    BATTERY_POLL_INTERVAL: float
NANO_CONFIG = NANOConfig(
    PORT="/dev/ttyAMA2",
    BAUD_RATE=9600,
    TIMEOUT=1,
    BATTERY_POLL_INTERVAL=5,
)

@dataclass
class LIDARConfig:
    PORT: str
    BAUD_RATE: int
    TIMEOUT: float

    BUFFER_SIZE: int
    SCAN_PACKET_SIZE: int
    SCAN_PACKETS_PER_TIME: int

    START_COMMAND: bytes
    STOP_COMMAND: bytes
    HEALTH_COMMAND: bytes
LIDAR_CONFIG = LIDARConfig(
    PORT="/dev/ttyUSB0",
    BAUD_RATE=460800,
    TIMEOUT=0.05,
    BUFFER_SIZE=4095,
    SCAN_PACKET_SIZE=5,
    SCAN_PACKETS_PER_TIME=200,
    START_COMMAND=b'\xA5\x20',
    STOP_COMMAND=b'\xA5\x25',
    HEALTH_COMMAND=b'\xA5\x52',
)

#### DEVICES

    

    
# -------------------------
# Logging Configuration
# -------------------------


# # -------------------------
# # Device Configuration
# # -------------------------

# # RP2040 Configuration (e.g., for motor control and odometry)
# RP2040_CONFIG = {
#     # Example parameters:
#     "BAUD_RATE": 115200,
#     "TIMEOUT": 0.1,
# }

# # NANO Configuration (e.g., for battery monitoring)
# NANO_CONFIG = {
#     # Example parameters:
#     "BAUD_RATE": 9600,
#     "TIMEOUT": 0.1,
# }

# # Lidar Configuration
# LIDAR_CONFIG = {
#     "PORT": "/dev/ttyUSB0",         # Serial port where the Lidar is connected
#     "BAUD_RATE": 460800,            # Baud rate for the Lidar connection
#     "TIMEOUT": 0.05,                # Serial read timeout
#     "SCAN_PACKET_SIZE": 5,          # Size of each scan packet (in bytes)
#     "SCAN_PACKETS_PER_TIME": 200    # How many packets to read per cycle
# }

# # -------------------------
# # Mapping and Localization
# # -------------------------
# GLOBAL_MAP_CONFIG = {
#     "INITIAL_SIZE_MM": 10000,  # Initial global map size in millimeters
#     "RESOLUTION": 100,         # Map resolution in millimeters per cell
# }

# # -------------------------
# # Miscellaneous
# # -------------------------
# # You can add additional global settings here, such as:
# # - Control loop timing
# # - Timeouts for network connections
# # - Other sensor-specific configurations

if __name__ == "__main__":
    # Quick test to print configuration values
    print("Network Configurations:")
    print(f"  HOST: {HOST}")
    print(f"  CMD_PORT: {CMD_PORT}")
    print(f"  MANUAL_PORT: {MANUAL_PORT}")
    print(f"  DATA_PORT: {DATA_PORT}")
    print("\nDevice Configurations:")
    print("  RP2040_CONFIG:", RP2040_CONFIG)
    print("  NANO_CONFIG:", NANO_CONFIG)
    print("  LIDAR_CONFIG:", LIDAR_CONFIG)
    print("\nMapping Configurations:")
    print("  GLOBAL_MAP_CONFIG:", GLOBAL_MAP_CONFIG)
