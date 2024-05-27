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
        self.socket.listen(1)
        print(f"[TCP SERVER] Listening on {self.hostname}:{self.port} ...")
        while self.running:
            try:
                self.connection, addr = self.socket.accept()
                print(f"[TCP SERVER] Connected with {addr}.")
                threading.Thread(target=self.handle_client, daemon=True).start()
            except socket.timeout:
                continue

    def handle_client(self):
        self.connected = True
        while self.running:
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
                break


    def send(self, lin_vel, ang_vel):
        if not self.connected:
            raise Exception("No client connected")
        self.connection.send(f"{lin_vel} {ang_vel}".encode())

    def end_connection(self): 
        self.connection.close()
        self.connected = False

    def stop(self):
        self.running = False
        self.socket.close()


class UDPServer(threading.Thread):
    def __init__(self, port):
        super().__init__(daemon=True)
        self.hostname = socket.gethostname()
        self.port = port
        self.queue = queue.Queue()
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.running = True
        self.connected = False
    
    def run(self):
        self.socket.bind((self.hostname, self.port))
        print(f"[UDP SERVER] Listening on {self.hostname}:{self.port}")
        while self.running:
            try:
                data, addr = self.socket.recvfrom(1024)
                if not self.connected and data.decode().strip() == "SYN":
                    self.socket.send("ACK".encode(), addr)
                    print(f"[UDP SERVER] Connected with {addr}")
                    self.connected = True

                elif self.connected and data.decode().strip() == "EXIT":
                    self.end_connection()
                    print("[UDP SERVER] Connection closed by client.\n")

                else:
                    self.queue.put(data.decode('utf-8'))

            except socket.error:
                self.connected = False
                print("[UDP SERVER] Socket error, connection lost.")
                continue

    def end_connection(self):
        self.connected = False

    def stop(self):
        self.running = False
        self.socket.close()
