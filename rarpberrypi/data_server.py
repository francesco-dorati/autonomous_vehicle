import threading
import socket

class DataServer(threading.Thread):
    def __init__(self, hostname, port, client_hostname, serial, frequency):
        super().__init__(daemon=True)
        self.hostname = hostname
        self.port = port
        self.client_hostname = client_hostname
        self.client_port = None

        self._stop_event = threading.Event()

        self.server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.server.bind((self.hostname, self.port))
        self.server.settimeout(1)
        print(f"\n[DATA SERVER] Listening on port {self.port}")

    def run(self) -> None:
        while not self._stop_event.is_set():

            if self.client_port is None:
                try:
                    data, addr = self.server.recvfrom(1024)
                    if addr[0] != self.client_hostname:
                        continue
                    self.client_port = addr[1]
                except socket.timeout: # TODO CHANGE
                    continue
            else:
                try:
                    data = self._get_data() # TODO CHANGE THIS
                    self.server.sendto(data, (self.client_hostname, self.client_port))

                except socket.error: # TODO CHANGE
                    self.stop()
                    break
            
            # delay
        
        def stop(self):
            self._stop_event.set()
            self.server.close()
            print("[DATA SERVER] Server stopped.")

        
