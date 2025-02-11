import serial
import threading
import time

from raspberry_pi.devices.device import Device
from raspberry_pi.utils.logger import get_logger, timing_decorator
from raspberry_pi.config import NANO_CONFIG

logger = get_logger(__name__)

class NANO(Device):
    """
    inherits from Device
    Static class for communication with the Arduino Nano.
    [lock]
    
    PUBLIC
        start() -> None
            [delay] starts serial connection
            raises ConnectionFailed: if starting failed

        stop() -> None
            [delay] stops serial connection
            raises ConnectionNotInitiated: if connection not initiated

        ping() -> bool
            [delay] checks connection with nano
            returns: ping successful

        get_battery() -> int
            [delay] gets battery voltage
            raises InvalidResponse: if invalid response from nano
            returns: battery voltage in mV [delay]

        start_sensors() -> None
            starts sensors
            raises SensorsMismatch: if sensors already running
            
        stop_sensors() -> None
            stops sensors
            raises SensorsMismatch: if sensors not running # TODO remove
        
        request_sensors() -> list
            [delay] requests sensor data
            raises SensorsMismatch: if sensors not running
            raises InvalidResponse: if invalid response from nano 
            returns: sensor data [mm] [fl, fr, rl, rr]

    PRIVATE
        _serial: serial.Serial
            serial connection

        _request_lock: threading.Lock
            lock for serial requests

        _sensors_running: bool
            flag for sensors running
    """
    _serial = None
    _request_lock = None
    _sensors_running = False  


    class InvalidResponse(Exception):
        pass
    class SensorsMismatch(Exception):           # TODO remove to start and stop sensors
        pass

    __battery_lock = threading.Lock()
    __battery_value = None
    __battery_polling_thread = None
    __battery_polling_stop = threading.Event()

    @staticmethod
    @timing_decorator
    def start():
        """
        Starts serial connection with nano
        Raises:
            ConnectionFailed: if starting failed
        """
        try:
            NANO._serial = serial.Serial(NANO_CONFIG.PORT, NANO_CONFIG.BAUD_RATE, timeout=NANO_CONFIG.TIMEOUT)
        except Exception as e:
            raise NANO.ConnectionFailed(f"Could not open serial port: {e}")

        NANO._request_lock = threading.Lock()
        NANO._sensors_running = False
        time.sleep(0.2)
        if not NANO.ping():
            NANO.stop()
            raise NANO.ConnectionFailed("Starting failed.")

        NANO.__start_battery_reader()
        logger.info("NANO connection established.")


    @staticmethod
    @timing_decorator
    def stop() -> None:
        """
        Stops serial connection with nano
        Raises:
          ConnectionNotInitiated: if connection not initiated
        """
        if not NANO._serial:
            raise NANO.ConnectionNotInitiated("Connection not initiated.")
        
        time.sleep(0.2)
        # stop sensors
        if NANO._sensors_running:
            NANO.stop_sensors()
        # stop battery reader
        NANO.__stop_battery_reader()
        # close serial
        if NANO._serial:
            NANO._serial.close()
        NANO._serial = None
        NANO._request_lock = None
        NANO._sensors_running = None


    @staticmethod
    @timing_decorator
    def ping() -> bool:
        """ PURE
        Send ping request to nano
        Returns:
            bool: ping successful
        """
        # request
        with NANO._request_lock:
            NANO._serial.write("P\n".encode())
            line = NANO._serial.readline()
        # validity check 
        if not line:
            return False
        res = line.decode().strip()
        return res == 'P'
    
    @staticmethod
    @timing_decorator
    def get_battery() -> int:
        """
        Get Battery mV
        Raises:
            InvalidResponse: if invalid response from nano
        Returns:
            int: battery voltage [mV]
        """
        with NANO.__battery_lock:
            return NANO.__battery_value
    
    @staticmethod
    @timing_decorator
    def start_sensors() -> None:
        """
        Start sensors
        Raises:
            SensorsMismatch: if sensors already running
        """
        if NANO._sensors_running:
            raise NANO.SensorsMismatch("Sensors already running.")
            
        # request
        NANO._serial.write("S1\n".encode())
        NANO._sensors_running = True

    @staticmethod
    @timing_decorator
    def stop_sensors() -> None:
        """
        Stop sensors
        Raises:
            SensorsMismatch: if sensors not running
        """
        if not NANO._sensors_running:
            raise NANO.SensorsMismatch("Sensors not running.")
        # request
        NANO._serial.write("S0\n".encode())
        NANO._sensors_running = False

    @staticmethod
    @timing_decorator
    def request_sensors() -> list:
        """
        Request distances from sensors
        Raises:
            SensorsNotRunning: if sensors not running
            InvalidResponse: if invalid response from nano
        Returns:
            list: list of distances from sensors [mm] [fl, fr, rl, rr]
        """
        # sensors running check
        if not NANO._sensors_running:
            NANO.stop()
            raise NANO.SensorsNotRunning("Sensors not running.")

        # request
        with NANO._request_lock:
            NANO._serial.write("SR\n".encode())
            line = NANO._serial.readline()

        # validity check
        if not line:
            raise NANO.InvalidResponse("No sensor response from nano.")
        try:
            fl, fr, rl, rr = map(int, line.decode().strip().split(" "))
        except:
            raise NANO.InvalidResponse("Invalid sensor response from nano.")
        return [fl, fr, rl, rr]

    @staticmethod
    def __start_battery_reader():
        """
        Start the battery polling thread.
        """
        if NANO.__battery_polling_thread is None:
            NANO.__battery_polling_stop.clear()
            NANO.__battery_polling_thread = threading.Thread(
                target=NANO.__battery_polling_loop, daemon=True
            )
            NANO.__battery_polling_thread.start()

    @staticmethod
    def __stop_battery_reader():
        """
        Stop the battery polling thread.
        """
        if NANO.__battery_polling_thread:
            NANO.__battery_polling_stop.set()
            NANO.__battery_polling_thread.join(timeout=1)
            NANO.__battery_polling_thread = None
    
    @staticmethod
    def __battery_polling_loop():
        """
        Background loop that polls the battery voltage and caches the result.
        """
        while not NANO.__battery_polling_stop.is_set():
            try:
                battery = NANO.__battery_request()  # blocking call that takes ~20ms
                with NANO.__battery_lock:
                    NANO.__battery_value = battery
            except Exception as e:
                # Optionally log the error.
                logger.error(f"Error polling battery voltage: {e}")
                pass
            time.sleep(NANO_CONFIG.BATTERY_POLL_INTERVAL)

    def __battery_request() -> int:
        # request
        with NANO._request_lock:
            NANO._serial.write("B\n".encode())
            line = NANO._serial.readline()
        # validity check
        if not line:
            raise NANO.InvalidResponse("No battery response from nano.")
        try:
            line = line.decode().strip()
            battery_mv = int(line)
        except:
            raise NANO.InvalidResponse(f"Invalid battery response from nano: \"{line}\"")
        return battery_mv