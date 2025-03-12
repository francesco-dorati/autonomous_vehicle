# Autonomous Mobile Robot
<!-- [manual interface](./img/manual_interface.png) -->
## Introduction
The aim of this project is to build an autonomous mobile robot capable of **localization**, **mapping** and **autonomous planning** in real-time.
This project is still under development.

### Main Components
- Raspberry Pi 
- Arduino nano
- Arduino RP2040 Connect
- RP Lidar
- [Full components list...](docs/compnents_list.md)

## Software Details
### Program Structure
![Program Structure](./img/main_structure.png)
#### Main Controller  (RaspberryPi)
Controls the central loop of the robot, handles connections with client and robot controllers.  
[[main.py](src/raspberry_pi/main.py), [manual.py](src/raspberry_pi/manual.py)]

#### Sensor Controller (Arduino RP2040)
Controls 4 distance sensors, 2 motor encoders and battery voltage.
Calculates robot odometry with motor encoders (expanding with IMU).
Connects to RaspberryPI with UART2 (serial).  
[[arduino code](src/rp2040/main_serial.ino),  [raspberrypi driver](src/raspberrypi/rp2040.py)]
```
RP2040 LOOP (50Hz):
- update battery
- update encoders and odometry
- update distance sensors

RP2040 ON_REQUEST:
- send battery voltage (V)
- send obstacle distances (cm, cm, cm, cm)
- send odometry (cm/s, deg/s, cm, cm, deg)
```
#### Motor Controller (Arduino Nano)
Controls DC motors.
Connects to RaspberryPI with UART0 (serial).  
[[arduino code](src/nano/main.ino),  [raspberrypi driver](src/raspberrypi/nano.py)]
```
NANO LOOP (50Hz):
- read from serial
- set motor powers
```
#### Developer Console (Client)
Graphical interface to allow the end-user to remotely control the robot while receiving camera streaming and sensors feedback.
Connects to RaspberryPI with multiple socket connections.

### Interfaces & Connections

#### Sockets (DevConsole <-> RaspberryPI):
- main: TCP (req/res)
- manual: UDP (bidirect)
- camera: UDP (monodirect)
```
MAIN
console (request) -->  raspberrypi (response)
"P"          -->    "P <battery_v>"             // Ping
"M <0/1>"    -->    "OK <manual_port>"          // Manual Start/Stop
"C <0/1>"    -->    "OK <camera_port>"          // Camera Start/Stop
"E"          -->    "OK"                        // Close Connection

MANUAL
console: "<boost> <x> <y>"                   // commands (boost, x, y ∈ [-1,0,1])
raspberrypi: "D <fl> <fr> <rl> <rr>",        // obstacle distance (cm, cm, cm, cm)
             "E <vx> <vt> <x> <y> <theta>",  // encoders odometry (cm/s, deg/s, cm, cm, deg)
```

#### Serial:
- arduino nano: UART0
- arduino rp2040: UART2






