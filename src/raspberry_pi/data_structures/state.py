# class State:
#     def __init__(self, position, velocity):
#         self.pos = position
#         self.vel = velocity
        


class Position:
    """
    Position
        x (int): mm
        y (int): mm
        th (int): mrad
    """
    def __init__(self, x, y, th):
        self.x = x
        self.y = y
        self.th = th

    def __str__(self):
        return f"x: {self.x}, y: {self.y}, th: {self.th}"
    
    

# class Velocity:
#     def __init__(self):
#         pass