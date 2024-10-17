# Autonomous Two-Wheeled Robot
## Introduction

### Key Features
- Manual control via a graphical interface with camera and sensors feedback.
- Odometry and obstacle detection capabilities for accurate positioning and movement.
- Working on expanding the project with LIDAR integration, autonomous mapping and real-time path planning capabilities.


### Main Components
- Raspberry Pi 
- Arduino nano
- Arduino RP2040 Connect
[Full components list...](schematics/compnents_list.md)

## Structure

### Main Controller  (RaspberryPi)
Controls main loop
[Main File](src/raspberry_pi/main.py)

### Sensors Controller (Arduino RP2040)
handles sensors
serial

### Actuators Controller (Arduino Nano)
handles actuators
serial

### Developer Console (Client)
interaction with robot
sockets



