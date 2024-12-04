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
        print(f"\n> Calling '{func.__qualname__}{args}'")
        ts = time.time()
        res = func(*args, **kwargs)
        dt = (time.time() - ts)*1000
        print(f"> returned {res}")
        print(f"> took: {dt:.2f} ms")
        return res
    return wrapper