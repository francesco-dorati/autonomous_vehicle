import socket

class AutonomousConsole:
    def __init__(self, hostname, port):
        self.hostname = hostname
        self.port = port

        print("\n[AUTO] Starting auto controller...")
        print(f"[AUTO] Connecting to {self.hostname}:{self.port}...")
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.connect((self.hostname, self.port))
            
            print("[AUTO] Connection established.")
            
        except socket.error:
            raise Exception("[AUTO] Connection failed.")
        
        print("\nAUTONOMOUS CONSOLE\n")


    def run(self):
        while True:
            command = input("AUTO> ").strip().split()

            if len(command) == 0:
                continue

            if command[0] == "exit" or command[0] == "EXIT":
                self.socket.send("EXIT".encode())
                self.socket.close()
                return

            if command[0] == "mv" or command[0] == "m": # mv 100 (cm)
                try:
                    dist_cm = int(command[1])
                    self.socket.send(f"m {dist_cm}".encode())
                    ack = self.socket.recv(1024).decode('utf-8')
                    print(ack)
                finally:
                    continue
  
            elif command[0] == "rot" or command[0] == "r": # rot 90 (deg)
                try:
                    angle_deg = int(command[1])
                    self.socket.send(f"r {angle_deg}".encode())
                    ack = self.socket.recv(1024).decode('utf-8')
                    print(ack)
                finally:
                    continue

            else: 
                continue