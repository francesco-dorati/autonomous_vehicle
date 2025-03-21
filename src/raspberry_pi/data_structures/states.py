from raspberry_pi.utils.utils import Utils
import struct
import numpy as np

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
        x (int): m
        y (int): m
        th (int): rad
    """
    def __init__(self, x, y, th):
        self.vect = np.array([x, y, th], dtype=np.float64)
    
    @property
    def x(self) -> np.float64:
        """x in meters"""
        return self.vect[0]
    @property
    def y(self) -> np.float64:
        """y in meters"""
        return self.vect[1]
    @property
    def th(self) -> np.float64:
        """th in radians"""
        return self.vect[2]
    
    def __str__(self):
        return f"x: {self.x}, y: {self.y}, th: {self.th}"
    def __repr__(self):
        return f"pos({self.x}, {self.y}, {self.th})"
    
    def get_point(self) -> CartPoint:
        return np.array([self.x, self.y], dtype=np.float64)
    
    def __eq__(self, other):
        if isinstance(other, Position):
            return self.x == other.x and self.y == other.y and self.th == other.th
        return False
    # def __add__(self, other):
    #     if isinstance(other, Position):
    #         return Position(self.x + other.x, self.y + other.y, Utils.normalize_mrad(self.th + other.th))
    #     return NotImplemented
    # def __sub__(self, other):
    #     if isinstance(other, Position):
    #         return Position(self.x - other.x, self.y - other.y, Utils.normalize_mrad(self.th - other.th))
    #     return NotImplemented
    

    
class State:
    """
    State
        x (int): m
        y (int): m
        th (int): rad
        v (int): m/s
        w (int): rad/s
    """
    def __init__(self, x=0, y=0, th=0, v=0, w=0):
        self.vect = np.array([x, y, th, v, w], dtype=np.float64)
    
    @property
    def x(self) -> np.float64:
        """x in meters"""
        return self.vect[0]
    @property
    def y(self) -> np.float64:
        """y in meters"""
        return self.vect[1]
    @property
    def th(self) -> np.float64:
        """th in radians"""
        return self.vect[2]
    @property
    def v(self) -> np.float64:
        """v in m/s"""
        return self.vect[3]
    @property
    def w(self) -> np.float64:
        """w in rad/s"""
        return self.vect[4]
  
    
    def __str__(self):
        return f"x: {self.x}, y: {self.y}, th: {self.th}, v: {self.v}, w: {self.w}"
    def __repr__(self):
        return f"state({self.x}, {self.y}, {self.th}, {self.v}, {self.w})"
    def get_position(self) -> Position:
        return Position(self.x, self.y, self.th)
    
# class Velocity:
#     def __init__(self):
#         pass