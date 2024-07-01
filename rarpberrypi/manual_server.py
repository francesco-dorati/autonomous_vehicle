import socket
import threading

class ManualServer(threading.Thread):
    def __init__(self, hostname, port, client_hostname):
        super().__init__(daemon=True)

        self.hostname = hostname
        self.port = port
        self.client_hostname = client_hostname

        self._stop_event = threading.Event()

        self.server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.server.bind((self.hostname, self.port))
        self.server.settimeout(1)
        print(f"\n[MANUAL SERVER] Listening on port {self.port}")

    def run(self):
        while not self._stop_event.is_set():
            try:
                data, addr = self.server.recvfrom(1024)
                if addr[0] != self.client_hostname:
                    print(f"[MANUAL SERVER] Received data from unknown client: {addr[0]} vs {self.client_hostname}")
                    continue

                print(f"[MANUAL SERVER] Received \"{data.decode()}\"")
                # lin_vel, ang_vel = self._calculate_speed(data.decode())
                # self.serial.send(lin_vel, ang_vel)

            except socket.timeout: # TODO
                continue
            
            except OSError:
                break
        
    def stop(self):
        self._stop_event.set()
        self.server.close()
        print("[MANUAL SERVER] Server stopped.")
    
    def _calculate_speed(self, data):
        pass