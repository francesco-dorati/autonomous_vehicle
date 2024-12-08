import time

class Utils:
    PI_MRAD = 3141.592653
    MRAD_DEG = 17.4532925199
    @staticmethod
    def normalize_mrad(mrad: int) -> int:
        return (mrad + Utils.PI_MRAD) % (2*Utils.PI_MRAD) - Utils.PI_MRAD

    @staticmethod
    def mrad_to_deg(mrad: int) -> float:
        return mrad / Utils.MRAD_DEG
    
    @staticmethod
    def deg_to_mrad(deg: float) -> int:
        return int(deg * Utils.MRAD_DEG)

def timing_decorator(func):
    def wrapper(*args, **kwargs):
        print(f"\n> Calling '{func.__qualname__}{args}'")
        ts = time.time()
        res = func(*args, **kwargs)
        dt = (time.time() - ts)*1000
        print(f"> returned {res}")
        print(f"> took: {dt:.2f} ms")
        return res
    return wrapper