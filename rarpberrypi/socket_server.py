import threading
import socket 
import queue

class TCPServer(threading.Thread):
    def __init__(self, port):
        super().__init__(daemon=True)
        self.hostname = socket.gethostname()
        self.port = port
        self.queue = queue.Queue()
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.running = True
        self.connected = False
    
    def run(self):
        self.socket.bind((self.hostname, self.port))
        self.server_socket.listen(1)
        print(f"TCP server listening on {self.hostname}:{self.port}")
        while self.running:
            try:
                self.connection, addr = self.socket.accept()
                print("Established a connection with", addr)
                threading.Thread(target=self.handle_client, daemon=True).start()
            except socket.timeout:
                continue

    def handle_client(self):
        self.connected = True
        while self.running:
            try:
                data = self.connection.recv(1024)
                if data:
                    self.queue.put(data.decode())
            except socket.error:
                break

        self.end_connection()

    def send(self, data):
        if not self.connected:
            raise Exception("No client connected")
        self.connection.send(data.encode())

    def end_connection(self): 
        self.connection.close()
        self.connected = False

    def stop(self):
        self.running = False
        self.socket.close()

class UDPServer(threading.Thread):
    def __init__(self, port):
        super().__init__(daemon=True)
        self.host = socket.gethostname()
        self.port = port
        self.queue = queue.Queue()
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.running = True
        self.connected = False
    
    def run(self):
        self.socket.bind((self.host, self.port))
        print(f"UDP server listening on {self.hostname}:{self.port}")
        while self.running:
            try:
                data, addr = self.server_socket.recvfrom(1024)
                self.queue.put(data.decode())
                self.connected = True
            except socket.error:
                continue

    def end_connection(self):
        self.connected = False

    def stop(self):
        self.running = False
        self.server_socket.close()
