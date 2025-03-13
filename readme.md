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
![program structure](./img/main_structure.png)


#### Robot Controller  (RaspberryPi)
Main controller of the robot, handles main logic and runs the control loop.  

##### Control Loop [4/5 Hz]
The control loop is responsible for the continuous operation of the robot, handling perception, planning, and control tasks.

1. **Perception**
    1. **State Prediction**:
        - Retreaves odometry data from Arduino RP2040
        - Uses an **Extended Kalman Filter (EKF)** to predict the new state (x, y, th, v, w).
    2. **Visual Odometry**:
        - Processes data from the Lidar to create a local map.
        - Uses **ICP** to compute map transformation.
    3. **State Update**:
        - Refines the predicted state using visual odometry data.
    4. **Mapping**:
         - If mapping is enabled, **expands the global map** with lidar data.

2. **Planning**
    1. **Control Commands**: Depending on the control type (position or velocity), sends appropriate commands to the RP2040 to move the robot.

The loop runs continuously at a specified interval, ensuring the robot can navigate and map its environment in real-time.  
[[robot.py](src/raspberry_pi/robot/robot.py)]

#### Motor Controller (Arduino RP2040) [50 Hz]
Handles velocity control of motors and wheel odomtery.
Receives commands from the Robot Controller and translates them into motor actions using a PID control.
Sends encoder data to the Robot Controller each loop.
The Arduino RP2040 communicates with the Raspberry Pi via UART2.
[[arduino code](src/arduino_rp2040/main.ino),  [raspberrypi driver](src/raspberrypi/rp2040.py)]


<!-- #### Motor Controller (Arduino Nano)

[[arduino code](src/nano/main.ino),  [raspberrypi driver](src/raspberrypi/nano.py)]
```
NANO LOOP (50Hz):
- read from serial
- set motor powers
``` -->
#### Developer Console (Client)
Graphical interface to allow the end-user to remotely control the robot while receiving camera streaming and sensors feedback.
Connects to RaspberryPI with multiple socket connections.


<!--
### Interfaces & Connections

#### Sockets (DevConsole <-> RaspberryPI):
- main: TCP (req/res)
- manual: UDP (bidirect)
- camera: UDP (monodirect)
```
MAIN
console (request) ->  raspberrypi (response)
"P"          ->    "P <battery_v>"             // Ping
"M <0/1>"    ->    "OK <manual_port>"          // Manual Start/Stop
"C <0/1>"    ->    "OK <camera_port>"          // Camera Start/Stop
"E"          ->    "OK"                        // Close Connection

MANUAL
console: "<boost> <x> <y>"                   // commands (boost, x, y ∈ [-1,0,1])
raspberrypi: "D <fl> <fr> <rl> <rr>",        // obstacle distance (cm, cm, cm, cm)
             "E <vx> <vt> <x> <y> <theta>",  // encoders odometry (cm/s, deg/s, cm, cm, deg)
``` ->
<- 
#### Serial:
- arduino nano: UART0
- arduino rp2040: UART2 -->






