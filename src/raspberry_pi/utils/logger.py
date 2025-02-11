import logging
import os
import sys
import time
from logging.handlers import RotatingFileHandler

# Configuration constants
LOG_LEVEL = logging.DEBUG  # Adjust as needed: DEBUG, INFO, WARNING, ERROR, CRITICAL
LOG_FORMAT = "[%(asctime)s] [%(levelname)s] [%(name)s]: %(message)s"
LOG_DATEFMT = "%Y-%m-%d %H:%M:%S"
LOG_FILE = os.path.join(os.path.dirname(__file__), "..", "logs", "app.log")
MAX_BYTES = 10 * 1024 * 1024  # 10 MB per log file
BACKUP_COUNT = 5             # Keep last 5 log files

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

def get_logger(name: str) -> logging.Logger:
    """
    Creates (or retrieves) and configures a logger with the given name.
    This logger will output to both the console and a rotating log file.
    
    Args:
        name (str): The name of the logger (usually __name__ in the caller module).
        
    Returns:
        logging.Logger: The configured logger.
    """
    logger = logging.getLogger(name)
    # Avoid adding duplicate handlers if already configured.
    if logger.hasHandlers():
        return logger

    logger.setLevel(LOG_LEVEL)

    # Create formatter
    formatter = logging.Formatter(fmt=LOG_FORMAT, datefmt=LOG_DATEFMT)

    # Console handler (prints to stdout)
    console_handler = logging.StreamHandler(sys.stdout)
    console_handler.setLevel(LOG_LEVEL)
    console_handler.setFormatter(formatter)
    logger.addHandler(console_handler)

    # Ensure the log directory exists
    log_dir = os.path.dirname(LOG_FILE)
    if not os.path.exists(log_dir):
        os.makedirs(log_dir)

    # File handler (rotating file handler)
    file_handler = RotatingFileHandler(LOG_FILE, maxBytes=MAX_BYTES, backupCount=BACKUP_COUNT)
    file_handler.setLevel(LOG_LEVEL)
    file_handler.setFormatter(formatter)
    logger.addHandler(file_handler)

    # Prevent log messages from being propagated to the root logger
    logger.propagate = False

    return logger

if __name__ == "__main__":
    # Quick test to verify logger configuration
    test_logger = get_logger("testLogger")
    test_logger.debug("This is a debug message.")
    test_logger.info("This is an info message.")
    test_logger.warning("This is a warning message.")
    test_logger.error("This is an error message.")
    test_logger.critical("This is a critical message.")
