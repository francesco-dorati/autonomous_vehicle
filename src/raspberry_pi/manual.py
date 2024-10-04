from enum import Enum
import socket

class ManualController:
    TRANSMITTER_DELAY = 0.2 # s

    class Speed(Enum):
        # index, pow, delta_pow
        STOP = 0, 0, 0
        SLOW = 1, 100, 100
        NORMAL = 2, 140, 40 
        FAST = 3, 180, 40

        def __init__(self, index, pow, delta_pow):
            self.index = index
            self.pow = pow
            self.delta_pow = delta_pow
        
    
    class Direction:
        def __init__(self, dir: str):
            self.x = 0
            self.y = 0
            for d in dir:
                if d == 'F':
                    self.x += 1
                elif d == 'B':
                    self.x -= 1
                elif d == 'L':
                    self.y += 1
                elif d == 'R':
                    self.y -= 1

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
        self.rp2040.request_data()

        speed, direction = self.__receive()
        if not speed or not direction:
            return

        # obstacle sensing
        if self.obstacle_sensing:
            speed, direction = self.__obstacle_sensing(speed, direction) # modify speed based on obstacle sensing

        pow_l, pow_r = self.__calculate_powers(speed, direction)

        if (time.time() - self.last_transmitted) >= TRANSMITTER_DELAY:
            self.__transmit(pow_l, pow_r)
    
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
            boost = self.Boost(int(d[0]))
            direction = self.Direction(d[1])
            return boost, direction

        except BlockingIOError:
            return None, None

    def __transmit(self):
        if self.client_addr == None:
            return
        message = f'P {self.rp2040.encoder_odometry.vx} {self.rp2040.encoder_odometry.vt} {self.rp2040.encoder_odometry.x} {self.rp2040.encoder_odometry.y} {self.rp2040.encoder_odometry.t}'
        message += f' D {self.rp2040.obstacle_distance.fl} {self.rp2040.obstacle_distance.fr} {self.rp2040.obstacle_distance.rl} {self.rp2040.obstacle_distance.rr}'
        self.server.sendto(message.encode(), self.client_addr)
        last_transmitted = time.time()

    def __obstacle_sensing(self, speed: Speed, direction: Direction) -> (Speed, Direction):
        if speed == None or direction == None:
            # only check distance if speed and direction are not None
            pass

        if direction.x > 0: # forward
            if self.rp2040.obstacle_distance.front_min() <= 30 and speed == self.Speed.FAST:
                speed = self.Speed.NORMAL
            if self.rp2040.obstacle_distance.front_min() <= 20 and speed == self.Speed.NORMAL:
                speed = self.Speed.SLOW
            if self.rp2040.obstacle_distance.front_min() <= 10:
                direction.x = 0
        elif direction.x < 0:
            if self.rp2040.obstacle_distance.back_min() <= 30 and speed == self.Speed.FAST:
                speed = self.Speed.NORMAL
            if self.rp2040.obstacle_distance.back_min() <= 20 and speed == self.Speed.NORMAL:
                speed = self.Speed.SLOW
            if self.rp2040.obstacle_distance.back_min() <= 10:
                direction.x = 0



    def __calculate_powers(self, speed: Speed, direction: Direction) -> (int, int):
        # calculate direction
        if direction.x == 0 and direction.y == 0:
            pow_l = 0
            pow_r = 0
        
        elif direction.y == 0:
            pow_l = speed.pow * direction.x
            pow_r = speed.pow * direction.x
        
        elif direction.x == 0:
            pow_l = -speed.pow * direction.y
            pow_r = speed.pow * direction.y

        else:
            pow_l = speed.pow * direction.x
            pow_r = speed.pow * direction.x
            pow_l += -speed.delta_pow * direction.y
            pow_r += speed.delta_pow * direction.y

        return pow_l, pow_r