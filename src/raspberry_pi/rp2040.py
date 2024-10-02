import smbus
import struct 
import math



class RP2040:
    class Battery:
        MEDIUM_V = 11.3
        LOW_V = 10.5
        CRITICAL_V = 9.8

        class Level(enum):
            CRITICAL = 0
            LOW = 1
            MEDIUM = 2
            HIGH = 3    

        def __init__(self):
            self.voltage = 0.0

        def reset(self):
            self.voltage = 0.0
        
        def level(self):
            if (self.voltage > 11.3):
                return self.Level.HIGH
            elif (self.voltage > 10.5):
                return self.Level.MEDIUM
            elif (self.voltage > 9.8):
                return self.Level.LOW
            else:
                return self.Level.CRITICAL
        
        def is_critical(self):
            return self.level() == self.Level.CRITICAL
        
    class Wheels: 
        def __init__(self):
            self.set(0.0, 0.0)
        def set(self, vl, vr):
            self.vl = vl 
            self.vr = vr
        def reset(self):
            self.set(0.0, 0.0)
    
    class Odometry:
        def __init__(self):
            self.set(0.0, 0.0, 0.0, 0.0, 0.0)
        def set(self, vx, vt, x, y, t):
            self.vx = vx    # cm/s
            self.vt = vt   # deg/s
            self.x = x    # cm
            self.y = y   # cm
            self.t = t  # deg
        def set_velocity(self, vx, vt):
            self.vx = vx
            self.vt = vt
        def set_position(self, x, y, t):
            self.x = x
            self.y = y
            self.t = t
        def reset(self):
            self.set(0.0, 0.0, 0.0, 0.0, 0.0)
        
    class SensorDistance:
        def __init__(self):
            self.set(0.0, 0.0, 0.0, 0.0)
        def set(self, fl, fr, bl, br):
            self.fl = fl
            self.fr = fr
            self.bl = bl
            self.br = br
        def reset(self):
            self.set(0.0, 0.0, 0.0, 0.0)
        def front_min(self):
            return min(self.fl, self.fr)
        def back_min(self):
            return min(self.bl, self.br)

    def __init__(self):
        self.bus = smbus.SMBus(1)  
        self.addr = 0x08 

        self.updated = False

        self.battery_on = False
        self.battery = Battery()

        self.encoder_on = False
        self.wheels = Wheels()
        self.encoders_odometry = Odometry()

        self.distance_on = False
        self.obstacle_distance = SensorDistance()
    
    def set_battery(self, on):
        # send command to rp2040
        self.bus.write_byte(self.addr, ord('B') if on else ord('b'))
        self.battery_on = on
        self.battery.voltage.reset()
    
    def set_encoder(self, on):
        # send command to rp2040
        self.bus.write_byte(self.addr, ord('E') if on else ord('e'))
        self.encoder_on = on
        self.wheels.reset()
        self.encoders_odometry.reset()

    def set_distance(self, on):
        # send command to rp2040
        self.bus.write_byte(self.addr, ord('D') if on else ord('d'))
        self.distance_on = on
        self.obstacle_distance.reset()

    def request_data(self):
        # send command to rp2040
        # ti amo cinci
        if self.updated:
            return
        else:
            self.updated = True

        raw_data = self.bus.read_i2c_block_data(self.addr, 32) 
        #print("Received: ", len(raw_data), raw_data) 
        unpacked_data = struct.unpack(b'<B9h3ib', bytes(raw_data))

        i = 0
        status = unpacked_data[i]
        self.battery_on = (status & 0b100) >> 2
        self.distance_on = (status & 0b010) >> 1
        self.encoders_on = (status & 0b001)
        i += 1

        self.battery.voltage = (unpacked_data[i]/1000) if self.battery_on else None
        i+=1

        self.obstacle_distance.set(updata_data[i]/10, unpacked_data[i+1]/10, unpacked_data[i+2]/10, unpacked_data[i+3]/10)
        i += 4

        self.wheels.set(unpacked_data[i]/10, unpacked_data[i+1]/10)
        i+=2

        self.encoders_odometry.set_velocity(unpacked_data[i]/10, (unpacked_data[i+1]*180)/(1000*math.pi))
        i+=2

        self.encoders_odometry.set_position(unpacked_data[i]/10, unpacked_data[i+1]/10, unpacked_data[i+2]*180/(1000*math.pi))
        i+=3
         




