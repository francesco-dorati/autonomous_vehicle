class State:
    def __init__(self, position, velocity):
        self.pos = position
        self.vel = velocity
        


class Position:
    """
    Position
        x (int): mm
        y (int): mm
        th (int): deg or mrad?????
    """
    def __init__(self, x, y, th):
        self.x = x
        self.y = y
        self.th = th

class Velocity:
    def __init__(self):
        pass