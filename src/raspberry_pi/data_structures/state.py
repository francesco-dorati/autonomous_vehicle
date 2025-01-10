# class State:
#     def __init__(self, position, velocity):
#         self.pos = position
#         self.vel = velocity
        

@dataclass
class Position:
    """
    Position
        x (int): mm
        y (int): mm
        th (int): mrad
    """
    def __init__(self, x, y, th):
        self.x = int(x)
        self.y = int(y)
        self.th = int(th)

    def __str__(self):
        return f"x: {self.x}, y: {self.y}, th: {self.th}"
    
    def __repr__(self):
        return f"pos({self.x}, {self.y}, {self.th})"
    
    

# class Velocity:
#     def __init__(self):
#         pass