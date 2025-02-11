
class CartPoint:
    """
    Point
        x (int): mm
        y (int): mm
    """
    def __init__(self, x, y):
        self.x = x
        self.y = y
    def __str__(self):
        return f"x: {self.x}, y: {self.y}"
    def __repr__(self):
        return f"point({self.x}, {self.y})"

class PolarPoint:
    """
    Polar Point
        r (int): mm
        th (int): mrad
    """
    def __init__(self, r, th):
        self.r = r
        self.th = th
    def __str__(self):
        return f"r: {self.r}, th: {self.th}"
    def __repr__(self):
        return f"point({self.r}, {self.th})"


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
    def get_point(self) -> CartPoint:
        return CartPoint(self.x, self.y)
    

    
    

# class Velocity:
#     def __init__(self):
#         pass