import smbus
import struct 
import math

class odometry:
    def __init__(self):
        self.vx: 0.0
        self.vt: 0.0
        self.x: 0.0
        self.y: 0.0
        self.z: 0.0
    

class RP2040:
    def __init__(self):
        self.bus = smbus.SMBus(1)  
        self.addr = 0x08 

        self.battery_on = False
        self.battery_voltage = 0.0

        self.encoder_on = False
        self.wheel_velocity = [0.0, 0.0]
        self.encoders_odometry = odometry()

        self.distance_on = False
        self.obstacle_distance = [0.0, 0.0, 0.0, 0.0]
    
    def set_battery(self, on):
        # send command to rp2040
        self.bus.write_byte(self.addr, ord('B') if on else ord('b'))
        self.battery_on = on
        self.battery_voltage = 0.0
     
    def set_encoder(self, on):
        # send command to rp2040
        self.bus.write_byte(self.addr, ord('E') if on else ord('e'))
        self.encoder_on = on
        self.wheel_velocity = [0.0, 0.0]
        self.encoders_odometry = odometry()

    def set_distance(self, on):
        # send command to rp2040
        if self.distance_on == on:
            return
        self.bus.write_byte(self.addr, ord('D') if on else ord('d'))
        self.distance_on = on
        self.obstacle_distance = [0.0, 0.0, 0.0, 0.0]

    def request_data(self):
        # send command to rp2040
        # ti amo cinci
        raw_data = self.bus.read_i2c_block_data(self.addr, 32) 
        print("Received: ", len(raw_data), raw_data) 
        unpacked_data = struct.unpack(b'<B9h3ib', bytes(raw_data))

        i = 0
        status = unpacked_data[i]
        self.battery_on = (status & 0b100) >> 2
        self.distance_on = (status & 0b010) >> 1
        self.encoders_on = (status & 0b001)
        i += 1
        print(f'b: {self.battery_on}, d: {self.distance_on}, e: {self.encoders_on}')

        self.battery_voltage = unpacked_data[i]/1000 if self.battery_on else None
        i+=1

        self.obstacle_distance[0] = unpacked_data[i]/10
        self.obstacle_distance[1] = unpacked_data[i+1]/10
        self.obstacle_distance[2] = unpacked_data[i+2]/10
        self.obstacle_distance[3] = unpacked_data[i+3]/10
        i += 4

        self.wheel_velocity[0] = unpacked_data[i]/10
        self.wheel_velocity[1] = unpacked_data[i+1]/10
        i+=2

        self.encoders_odometry.vx = unpacked_data[i]/10
        self.encoders_odometry.vt = unpacked_data[i+1]*180/(1000*math.pi),
        self.encoders_odometry.x = unpacked_data[i+2]/10,
        self.encoders_odometry.y = unpacked_data[i+3]/10,
        self.encoders_odometry.theta = unpacked_data[i+4]*180/(1000*math.pi)
        i+=5




