import threading
import socket 
import queue
import time


class MainServer(threading.Thread):
    def __init__(self, hostname, port):
        super().__init__(daemon=True)
        self.hostname = hostname
        self.port = port

        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind((self.hostname, self.port))
        print(f"[MAIN SERVER] Listening on {self.hostname}:{self.port}")

        self.in_queue = queue.Queue()
        self.out_queue = queue.Queue()

    def run(self):
        while True:
            data, addr = self.socket.recvfrom(1024)
            self.in_queue.put(data.decode())

    def send(self, data):
        self.socket.sendto(data.encode(), (self.hostname, self.port))

    def stop_server(self):
        self.socket.close()
        
class TCPServer(threading.Thread):
    def __init__(self, hostname, port):
        super().__init__(daemon=True)
        self.running = True
        self.connected = False
        self.hostname = hostname
        self.port = port
        self.queue = queue.Queue()

        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.bind((self.hostname, self.port))
        self.socket.listen(1)
        print(f"[TCP SERVER] Listening on {self.hostname}:{self.port}")

    def run(self):
        while True:
            if self.running:
                try:
                    self.connection, addr = self.socket.accept()
                    print(f"[TCP SERVER] Connected with {addr}.")
                    threading.Thread(target=self.handle_client, daemon=True).start()

                except socket.timeout:
                    continue
            else:
                time.sleep(1)

    def handle_client(self):
        self.connected = True
        while self.running and self.connected:
            try:
                data = self.connection.recv(1024)
                if not data:
                    continue
                
                if data.decode().strip() == "EXIT":
                    self.end_connection()
                    print(f"[TCP SERVER] Connection closed by client.")
                    break

                self.queue.put(data.decode())

            except socket.error:
                self.end_connection()
                raise Exception(f"[TCP SERVER] Connection closed by error.")


    # def send(self, lin_vel, ang_vel):
    #     if not self.connected:
    #         raise Exception("No client connected")
    #     self.connection.send(f"{lin_vel} {ang_vel}".encode())

    def end_connection(self): 
        self.connected = False
        self.connection.close()

    def block(self):
        if self.connected:
            raise Exception("Cannot block socket while it is still connected.")
        self.running = False
        
    def resume(self):
        self.running = True

    def stop_server(self):
        self.running = False
        self.socket.close()


class ManualServer(threading.Thread):
    def __init__(self, hostname, port):
        super().__init__(daemon=True)
        self.hostname = hostname
        self.port = port

        self.in_queue = queue.Queue()
        self.out_queue = queue.Queue()

        self.running = True
        self.connected = False
        self.connection_addr = None

        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind((self.hostname, self.port))
        print(f"[UDP SERVER] Listening on {self.hostname}:{self.port}")
    
    def run(self):
        while True:
            if self.running:
                try:
                    data, addr = self.socket.recvfrom(1024)

                    # SYN-ACK
                    if not self.connected and data.decode().strip() == "SYN":
                        self.socket.sendto("ACK".encode(), addr)
                        print(f"[UDP SERVER] Connected with {addr}")
                        self.connected = True
                        self.connection_addr = addr

                    # EXIT
                    elif self.connected and data.decode().strip() == "EXIT":
                        self.end_connection()
                        print("[UDP SERVER] Connection closed by client.\n")

                    # DATA
                    elif self.connected:
                        self.queue.put(data.decode())

                except socket.error:
                    self.connected = False
                    print("[UDP SERVER] Socket error, connection lost.")
                    continue
            else:
                time.sleep(1)

    def send(self, data):
        self.socket.sendto(data.encode(), self.connection_addr)

    def end_connection(self):
        self.connected = False
        self.connection_addr = None

    def block(self):
        if self.connected:
            raise Exception("Cannot block socket while it is still connected.")
        self.running = False
        
    def resume(self):
        self.running = True

    def stop_server(self):
        self.running = False
        self.socket.close()
