from abc import ABC, abstractmethod

class Device(ABC):
    
    class ConnectionFailed(Exception):
        pass

    class ConnectionNotInitiated(Exception):
        pass

    @abstractmethod
    def start():
        """
        Must start the device
        Raises:
            ConnectionFailed: if starting failed
        """
        pass

    @abstractmethod
    def stop():
        """
        Must stop the device
        """
        pass

    @abstractmethod
    def ping() -> bool:
        """ Must return if device is running"""
        pass