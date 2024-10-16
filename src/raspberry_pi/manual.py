from enum import Enum
import socket
import time

class ManualController:
    TRANSMITTER_DELAY = 0.2 # s
    class Command:
        BOOST_POW = {'lin': 200, 'rot': 150, 'delta': 50}
        NORMAL_POW = {'lin': 150, 'rot': 120, 'delta': 40}
        SHIFT_POW = {'lin': 110, 'rot': 100, 'delta': 30}

        def __init__(self, vel, x, y):
            self.vel = vel
            self.x = x
            self.y = y
            
        
        def calculate_powers(self):
            # calculate direction
            p = self._pow()

            if self.x == 0 and self.y == 0:
                return 0, 0
            
            elif self.y == 0: # front/back
                pow_l = p['lin'] * self.x
                pow_r = p['lin'] * self.x
            
            elif self.x == 0: # rotate
                pow_l = -p['rot'] * self.y
                pow_r = p['rot'] * self.y

            else: # diagonal
                pow_l = p['lin'] * self.x - p['delta'] * self.y
                pow_r = p['lin'] * self.x + p['delta'] * self.y

            return pow_l, pow_r

        
        def _pow(self):
            if self.vel == 1:
                return self.BOOST_POW
            elif self.vel == 0:
                return self.NORMAL_POW
            elif self.vel == -1:
                return self.SHIFT_POW


    # class Speed(Enum):
    #     # index, pow, delta_pow
    #     STOP = 0, 0, 0
    #     SLOW = 1, 100, 100
    #     NORMAL = 2, 140, 40 
    #     FAST = 3, 180, 40

    # #     def __init__(self, index, pow, delta_pow):
    #         self.index = index
    #         self.pow = pow
    #         self.delta_pow = delta_pow
        
    
    # class Direction:
    #     def __init__(self, dir: str):
    #         self.x = 0
    #         self.y = 0
    #         for d in dir:
    #             if d == 'F':
    #                 self.x += 1
    #             elif d == 'B':
    #                 self.x -= 1
    #             elif d == 'L':
    #                 self.y += 1
    #             elif d == 'R':
    #                 self.y -= 1

    def __init__(self, rp2040, nano, host, port):
        self.rp2040 = rp2040
        self.nano = nano
        self.host = host
        self.port = port

        self.server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.server.bind((self.host, self.port))
        self.server.setblocking(False)
        
        self.client_addr = None
        self.last_received = 0
        self.last_transmitted = 0


        self.obstacle_sensing = False
        # setup sensors and encoders
        self.rp2040.set_encoder(True)
        self.rp2040.set_distance(True)


    def compute(self):
        t_request = time.time()
        self.rp2040.request_data()
        dt_request = time.time() - t_request

        t_receive = time.time()
        command = self.__receive()
        dt_receive = time.time() - t_receive

        # obstacle sensing
        # if self.obstacle_sensing:
        #     speed, direction = self.__obstacle_sensing(speed, direction) # modify speed based on obstacle sensing
        dt_compute = 0
        dt_transmit = 0
        if command != None:
            print(f"Command: {command.vel} {command.x} {command.y}")
            t_compute = time.time()
            pow_l, pow_r = command.calculate_powers()
            dt_compute = time.time() - t_compute
            print(pow_l, pow_r)
            t_transmit = time.time()
            self.nano.send_power(pow_l, pow_r)
            dt_transmit = time.time() - t_transmit
        
        # else:
            # print('NONE')
            # self.nano.set_powers(pow_l, pow_r)

        print(f"M time rp20:{(dt_request*1000):.1f}\trec:{(dt_receive*1000):.1f}\tcomp:{(dt_compute*1000):.1f}\ttransm:{(dt_transmit*1000):.1f} ms")

        if (time.time() - self.last_transmitted) >= self.TRANSMITTER_DELAY:
            self.__transmit()
    
    def stop(self):
        self.nano.send_power(0, 0)
        self.rp2040.set_encoder(False)
        self.rp2040.set_distance(False)
        self.server.close()
        self.server = None

    def __receive(self) -> (int, str):
        try:
            data, addr = self.server.recvfrom(32)
            if data and self.client_addr == None:
                self.client_addr = addr
            # parse data
            d = data.decode().split(' ')
            vel_axis = int(d[0])
            x_axis = int(d[1])
            y_axis = int(d[2])
            c = self.Command(vel_axis, x_axis, y_axis)
            return c

        except BlockingIOError:
            return None

    def __transmit(self):
        if self.client_addr == None:
            return
        message = f'E {self.rp2040.encoders_odometry.vx} {self.rp2040.encoders_odometry.vt} {self.rp2040.encoders_odometry.x} {self.rp2040.encoders_odometry.y} {self.rp2040.encoders_odometry.t}\n'
        message += f'D {self.rp2040.obstacle_distance.fl} {self.rp2040.obstacle_distance.fr} {self.rp2040.obstacle_distance.rl} {self.rp2040.obstacle_distance.rr}'
        self.server.sendto(message.encode(), self.client_addr)
        self.last_transmitted = time.time()

    # def __obstacle_sensing(self, speed: Speed, direction: Direction) -> (Speed, Direction):
    #     if speed == None or direction == None:
    #         # only check distance if speed and direction are not None
    #         pass

    #     if direction.x > 0: # forward
    #         if self.rp2040.obstacle_distance.front_min() <= 30 and speed == self.Speed.FAST:
    #             speed = self.Speed.NORMAL
    #         if self.rp2040.obstacle_distance.front_min() <= 20 and speed == self.Speed.NORMAL:
    #             speed = self.Speed.SLOW
    #         if self.rp2040.obstacle_distance.front_min() <= 10:
    #             direction.x = 0
    #     elif direction.x < 0:
    #         if self.rp2040.obstacle_distance.back_min() <= 30 and speed == self.Speed.FAST:
    #             speed = self.Speed.NORMAL
    #         if self.rp2040.obstacle_distance.back_min() <= 20 and speed == self.Speed.NORMAL:
    #             speed = self.Speed.SLOW
    #         if self.rp2040.obstacle_distance.back_min() <= 10:
    #             direction.x = 0
