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

# def timing_decorator(func):
#     def wrapper(*args, **kwargs):
#         print(f"\n> Calling '{func.__qualname__}{args}'")
#         ts = time.time()
#         res = func(*args, **kwargs)
#         dt = (time.time() - ts)*1000
#         print(f"> [] returned {res}")
#         print(f"> took: {dt:.2f} ms")
#         return res
#     return wrapper


# Global variable to track call depth
call_depth = 0

def timing_decorator(func):
    def wrapper(*args, **kwargs):
        global call_depth
        
        # Indentation based on call depth
        indent = '  ' * call_depth
        call_depth += 1
        
        # Log function call
        arg_list = ', '.join([repr(arg) for arg in args] +
                             [f"{k}={repr(v)}" for k, v in kwargs.items()])
        print(f"{indent}> Calling '{func.__qualname__}({arg_list})'")
        
        # Time the function execution
        ts = time.time()
        try:
            res = func(*args, **kwargs)
        except Exception as e:
            print(f"{indent}! Exception in '{func.__qualname__}': {e}")
            call_depth -= 1
            raise
        dt = (time.time() - ts) * 1000
        
        # Log function result
        print(f"{indent}< Returned: {repr(res)}")
        print(f"{indent}- Took: {dt:.2f} ms")
        
        call_depth -= 1
        return res
    return wrapper