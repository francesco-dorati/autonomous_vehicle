# from smbus2 import SMBus
import serial
import struct 
import math
import time
from enum import Enum


class RP2040_I2C:
    class Battery:
        MEDIUM_V = 11.3
        LOW_V = 10.5
        CRITICAL_V = 9.8

        class Level(Enum):
            NULL = -1
            CRITICAL = 0
            LOW = 1
            MEDIUM = 2
            HIGH = 3    

        def __init__(self):
            self.voltage = None

        def reset(self):
            self.voltage = None
        
        def level(self):
            if not self.voltage:
                return self.Level.NULL
                
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
        self.bus = SMBus(1)  
        self.addr = 0x08 
        # self.bus._set_clock(400000)


        self.updated = False

        self.battery_on = False
        self.battery = self.Battery()

        self.encoder_on = False
        self.wheels = self.Wheels()
        self.encoders_odometry = self.Odometry()

        self.distance_on = False
        self.obstacle_distance = self.SensorDistance()
    
    def set_battery(self, on):
        # send command to rp2040
        self.bus.write_byte(self.addr, ord('B') if on else ord('b'))
        self.battery_on = on
        self.battery.reset()
    
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

        t_request = time.time()
        # raw_data = self.bus.read_i2c_block_data(self.addr, 0, 32) 
        raw_data = self.bus.read_i2c_block_data(self.addr, 0, 1) 
        dt_request = time.time() - t_request
        print(f"R {(dt_request*1000):.1f}")
        return
        #print("Received: ", len(raw_data), raw_data) 
        t_unpack = time.time()
        unpacked_data = struct.unpack(b'<B9h3ib', bytes(raw_data))
        dt_unpack = time.time() - t_unpack

        t_compute = time.time()
        i = 0
        status = unpacked_data[i]
        self.battery_on = (status & 0b100) >> 2
        self.distance_on = (status & 0b010) >> 1
        self.encoders_on = (status & 0b001)
        i += 1

        self.battery.voltage = (unpacked_data[i]/1000) if self.battery_on else None
        i+=1

        self.obstacle_distance.set(unpacked_data[i]/10, unpacked_data[i+1]/10, unpacked_data[i+2]/10, unpacked_data[i+3]/10)
        i += 4

        self.wheels.set(unpacked_data[i]/10, unpacked_data[i+1]/10)
        i+=2

        self.encoders_odometry.set_velocity(unpacked_data[i]/10, (unpacked_data[i+1]*180)/(1000*math.pi))
        i+=2

        self.encoders_odometry.set_position(unpacked_data[i]/10, unpacked_data[i+1]/10, unpacked_data[i+2]*180/(1000*math.pi))
        i+=3
        dt_compute = time.time() - t_compute
        print(f"R {(dt_request*1000):.1f}\t{(dt_unpack*1000):.1f}\t{(dt_compute*1000):.1f}")
         

class RP2040_SER:
    class Battery:
        MEDIUM_V = 11.3
        LOW_V = 10.5
        CRITICAL_V = 9.8

        class Level(Enum):
            NULL = -1
            CRITICAL = 0
            LOW = 1
            MEDIUM = 2
            HIGH = 3    

        def __init__(self):
            self.voltage = None

        def reset(self):
            self.voltage = None
        
        def level(self):
            if not self.voltage:
                return self.Level.NULL
                
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
        def set(self, fl, fr, rl, rr):
            self.fl = fl # 
            self.fr = fr
            self.rl = rl
            self.rr = rr
        def reset(self):
            self.set(0.0, 0.0, 0.0, 0.0)
        def front_min(self):
            return min(self.fl, self.fr)
        def back_min(self):
            return min(self.rl, self.rr)

    def __init__(self):
        print("SERIAL SETUP")
        self.ser = serial.Serial('/dev/ttyAMA2', 115200, timeout=0.05)
        self.updated = False

        self.battery_on = False
        self.battery = self.Battery()

        self.encoder_on = False
        self.encoders_odometry = self.Odometry()

        self.distance_on = False
        self.obstacle_distance = self.SensorDistance()
        print("RP2040 READY")
    
    def set_battery(self, on):
        # send command to rp2040
        s = f"SB{1 if on else 0}"
        self.ser.write(s.encode())
        self.battery_on = on
        self.battery.reset()
        print("Set Battery ok")
    
    def set_encoder(self, on):
        # send command to rp2040
        s = f"SE{1 if on else 0}"
        self.ser.write(s.encode())
        self.encoder_on = on
        self.encoders_odometry.reset()

    def set_distance(self, on):
        # send command to rp2040
        s = f"SD{1 if on else 0}"
        self.ser.write(s.encode())
        self.distance_on = on
        self.obstacle_distance.reset()

    def request_data(self):
        # send command to rp2040
        # ti amo cinci
        if self.updated:
            return
        else:
            self.updated = True

        print("Data request")
        t_request = time.time()
        self.ser.flush()
        self.ser.write(b'R*')
        dt_request = time.time() - t_request
        print(f"Request took {(dt_request*1000):.1f} ms")

        t_response = time.time()
        l1 = self.ser.readline()
        l2 = self.ser.readline()
        l3 = self.ser.readline()
        dt_response = time.time() - t_response
        print(f"Response took {(dt_response*1000):.1f} ms")

        t_decode = time.time()
        for line in [l1, l2, l3]:
            l = line.decode().strip().split(' ')
            print("Received line: ", l)  
            
            if l[0] == 'B': # Battery
                self.battery_on = (int(l[1]) == 1)
                if self.battery_on:
                    try:
                        self.battery.voltage = float(l[2])
                    except ValueError:
                        self.battery.reset()
                else:
                    self.battery.reset()
            elif l[0] == 'D': # Distance
                self.distance_on = (int(l[1]) == 1)
                if self.distance_on:
                    try:
                        self.obstacle_distance.set(float(l[2]), float(l[3]), float(l[4]), float(l[5]))
                    except ValueError:
                        self.obstacle_distance.reset()
                else:
                    self.obstacle_distance.reset()
            elif l[0] == 'E': # Encoders
                self.encoder_on = (int(l[1]) == 1)
                if self.encoder_on:
                    try:
                        self.encoders_odometry.set_velocity(float(l[2]), float(l[3]))
                        self.encoders_odometry.set_position(float(l[4]), float(l[5]), float(l[6]))
                    except ValueError:
                        self.encoders_odometry.reset()
                else:
                    self.encoders_odometry.reset()
        dt_decode = time.time() - t_decode
        print(f"Decode took {(dt_decode*1000):.1f} ms")


