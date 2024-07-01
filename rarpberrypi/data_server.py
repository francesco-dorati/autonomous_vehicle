import threading
import socket
import time

class DataServer(threading.Thread):
    def __init__(self, hostname, port, client_hostname, serial):
        super().__init__(daemon=True)
        self.hostname = hostname
        self.port = port
        self.client_hostname = client_hostname
        self.client_port = None

        self._stop_event = threading.Event()

        self.server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.server.bind((self.hostname, self.port))
        self.server.settimeout(1)
        self.rate_s = 500
        print(f"\n[DATA SERVER] Listening on port {self.port}")

    def set_rate(self, rate_ms: int):
        self.rate_ms = rate_ms

    def run(self) -> None:
        while not self._stop_event.is_set():
            if self.client_port is None:
                try:
                    data, addr = self.server.recvfrom(1024)
                    if addr[0] != self.client_hostname:
                        print(f"[DATA SERVER] Received data from unknown client: {addr[0]} instead of {self.client_hostname}")
                        continue

                    if data.decode() == "START":
                        self.client_port = addr[1]
                        self.server.sendto("OK".encode(), (self.client_hostname, self.client_port))
                except:
                    continue

            else:
                t_start = time.time()
                # send to serial
                # receive from serial
                
                dt_ms = (time.time() - t_start)*1000
                if dt_ms < self.rate_ms:
                    time.sleep((self.rate_ms - dt_ms)/1000)

        
    def stop(self):
        self.client_port = None
        self._stop_event.set()
        self.server.close()
        print("[DATA SERVER] Server stopped.")

        
