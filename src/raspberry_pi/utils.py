import time
class Utils:
    @staticmethod
    def normalize_mrad(mrad: int) -> int:
        pass

    @staticmethod
    def mrad_to_deg(mrad: int) -> float:
        pass

    @staticmethod
    def deg_to_mrad(deg: float) -> int:
        pass

def timing_decorator(func):
    def wrapper(*args, **kwargs):
        print(f"Calling '{func.__name__}' with arguments {args} and {kwargs}")
        ts = time.time()
        res = func(*args, **kwargs)
        dt = (time.time() - ts)*1000
        print(f"'{func.__name__}' returned {res}")
        print(f"took: {dt} ms\n")
        return res
    return wrapper