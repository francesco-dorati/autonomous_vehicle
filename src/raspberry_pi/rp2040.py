import smbus
import struct 

class RP2040:
    def __init__(self):
        self.battery_on = False
        self.encoder_on = False
        self.distance_on = False
        self.bus = smbus.SMBus(1)  
        self.addr = 0x08 
    
    def set_battery(self, on):
        # send command to rp2040
        if self.battery_on == on:
            return
        self.bus.write_byte(self.addr, 'B' if on else 'b')
        self.battery_on = on
    
    def set_encoder(self, on):
        # send command to rp2040
        if self.encoder_on == on:
            return
        self.bus.write_byte(self.addr, 'E' if on else 'e')
        self.encoder_on = on

    def set_distance(self, on):
        # send command to rp2040
        if self.distance_on == on:
            return
        self.bus.write_byte(self.addr, 'D' if on else 'd')
        self.distance_on = on

    def request_data(self):
        # send command to rp2040
        # ti amo cinci
        raw_data = self.bus.read_i2c_block_data(self.addr, 0, 33)            
        unpacked_data = struct.unpack('<3c9h3i', raw_data)

        i = 0
        battery_on = unpacked_data[0] == b'B'
        distance_on = unpacked_data[2] == b'D'
        encoder_on = unpacked_data[1] == b'E'
        i += 3

        self.battery_voltage_v = unpacked_data[i]/1000 if battery_on else None
        i+=1

        self.obstacle_distance_cm = {
            'FL': unpacked_data[i]/10,
            'FR': unpacked_data[i+1]/10,
            'RL': unpacked_data[i+2]/10,
            'RR': unpacked_data[i+3]/10 
        } if distance_on else None
        i+=4

        self.wheel_velocity_cms = {
            'L': unpacked_data[i],
            'R': unpacked_data[i+1]
        } if encoder_on else None
        i+=2

        self.robot_state = {
            'vx_cms': unpacked_data[i]/10,
            'vy_deg/s': unpacked_data[i+1]*180/(1000*math.pi),
            'x_cm': unpacked_data[i+2]/10,
            'y_cm': unpacked_data[i+3]/10,
            'theta_deg': unpacked_data[i+4]*180/(1000*math.pi)
        } if encoder_on else None
        i+=5




