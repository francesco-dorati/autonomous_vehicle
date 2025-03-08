from raspberry_pi.utils.utils import Utils
import struct

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
        return f"CartPoint({self.x:.2f}, {self.y:.2f})"
    def get_position(self, th: int = 0):
        return Position(self.x, self.y, 0)

class PolarPoint:
    """
    Polar Point
        r (int): mm
        th (int): mrad
    """
    def __init__(self, r, th):
        self.r = r
        self.th = Utils.normalize_mrad(th)
    def __str__(self):
        return f"r: {self.r}, th: {self.th}"
    def __repr__(self):
        return f"PolarPoint({self.r:.2f}, {self.th:.2f})"


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
    def __eq__(self, other):
        if isinstance(other, Position):
            return self.x == other.x and self.y == other.y and self.th == other.th
        return False
    def __add__(self, other):
        if isinstance(other, Position):
            return Position(self.x + other.x, self.y + other.y, Utils.normalize_mrad(self.th + other.th))
        return NotImplemented
    def __sub__(self, other):
        if isinstance(other, Position):
            return Position(self.x - other.x, self.y - other.y, Utils.normalize_mrad(self.th - other.th))
        return NotImplemented
    

    
    

# class Velocity:
#     def __init__(self):
#         pass