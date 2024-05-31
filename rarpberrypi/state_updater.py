import threading
import socket
import time

UPDATE_FREQUENCY = 10
UPDATE_TIME = (1/UPDATE_FREQUENCY)

class StateUpdater(threading.Thread):
    def __init__(self, hostname, port, state, serial):
        super().__init__(daemon=True)
        self.hostname = hostname
        self.port = port
        self.state = state
        self.serial = serial

        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.bind((self.hostname, self.port))
        self.connection = None
        print(f"[STATE UPDATER] Listening on {self.hostname}:{self.port}")
    
    def run(self):
        last_transmit_time = 0
        while True:
            if self.connection is None:
                self.connection, addr = self.socket.accept()
                print(f"[STATE UPDATER] Connected to {addr}")
                continue

            data = self.serial.read()
            vel, pos, dist = self.interpret_data(data)
            self.state.update(vel, pos, dist)

            if time.time() - last_transmit_time > UPDATE_TIME:
                self.connection.send(self.state.encode())
                last_transmit_time = time.time()

    def interpret_data(self):
        pass