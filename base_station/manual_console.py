import time
import keyboard
import json
import socket
import queue
import threading

MANUAL_FREQ = 10 # Hz
MANUAL_TAO = (1/MANUAL_FREQ) # s

class ManualConsole:
    def __init__(self, hostname, port):
        self.hostname = hostname
        self.port = port
        self.in_buffer = queue.Queue()

        print("\n[MANUAL] Starting manual controller...")
        print(f"[MANUAL] Connecting to {self.hostname}:{self.port}...")
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.socket.sendto("SYN\n".encode(), (self.hostname, self.port))
            ack, _ = self.socket.recvfrom(1024)
            if ack.decode().strip() == "ACK":
                print("[MANUAL] Connection established.")
            else:
                raise Exception(f"[MANUAL] Connection failed. Received '{ack}'.")
            threading.Thread()
            
        except socket.error:
            raise Exception("[MANUAL] Connection failed.")
        
        print("\nMANUAL CONSOLE\n")
        print("         W: Forward")
        print("A: Left  S: Backward  D: Right\n")
        print("(esc) or (space) to exit.")
        print("\n\n\n\n\n\n\n\n\n\n")

    def run(self):
        receiver = threading.Thread(target=self.receive_data, daemon=True)
        receiver.start()

        try:
            while True:
                start_time = time.time()
                input_buffer = []

                if keyboard.is_pressed('esc') or keyboard.is_pressed("space"):
                    self.socket.sendto("EXIT".encode(), (self.hostname, self.port))
                    self.socket.close()
                    return
                
                # read keypresses
                if keyboard.is_pressed('w'):
                    input_buffer.append("f") # forward
                if keyboard.is_pressed("s"):
                    input_buffer.append("b") # backward
                if keyboard.is_pressed("a"):
                    input_buffer.append("l") # left
                if keyboard.is_pressed("d"):
                    input_buffer.append("r") # right
                
                # buffer processing
                if "f" in input_buffer and "b" in input_buffer:
                    input_buffer.remove("f")
                    input_buffer.remove("b")
                if "l" in input_buffer and "r" in input_buffer:
                    input_buffer.remove("l")
                    input_buffer.remove("r")
                
                if not self.in_buffer.empty():
                    data = self.in_buffer.get()
                    self._print_direction(input_buffer, data)

                # send buffer
                self.socket.sendto(''.join(input_buffer).encode('utf-8'), (self.hostname, self.port))

                # wait until next iteration
                dt = time.time() - start_time
                if dt < MANUAL_TAO:
                    time.sleep(MANUAL_TAO - dt) 
        except:
            self.socket.close()
            return

        receiver.stop()

    def receive_data(self):
        while True:
            data, _ = self.socket.recvfrom(1024)
            if data:
                data = json.loads(data.decode())
                self.in_buffer.put(data)


    def _print_direction(self, input_buffer, data):

            arrow_up = '\u2191' if "f" in input_buffer else ' '
            arrow_down = '\u2193' if "b" in input_buffer else ' '
            arrow_left = '\u2190' if "l" in input_buffer else ' '
            arrow_right = '\u2192' if "r" in input_buffer else ' '

            print('\r' + ' ' * 80 + '\r ', end='')
            print('\x1b[A' + '\r' + ' ' * 80 + '\r', end='')
            print('\x1b[A' + '\r' + ' ' * 80 + '\r', end='')
            print('\x1b[A' + '\r' + ' ' * 80 + '\r', end='')
            print('\x1b[A' + '\r' + ' ' * 80 + '\r', end='')
            print('\x1b[A' + '\r' + ' ' * 80 + '\r', end='')
            print('\x1b[A' + '\r' + ' ' * 80 + '\r', end='')
            print('\x1b[A' + '\r' + ' ' * 80 + '\r', end='')
            print('\x1b[A' + '\r' + ' ' * 80 + '\r', end='')
            print('\x1b[A' + '\r' + ' ' * 80 + '\r', end='')
            print('\x1b[A' + '\r' + ' ' * 80 + '\r', end='')
            print('\x1b[A' + '\r' + ' ' * 80 + '\r', end='')
            print('\x1b[A' + '\r' + ' ' * 80 + '\r', end='')
            print('\x1b[A' + '\r' + ' ' * 80 + '\r', end='')
            print('\x1b[A' + '\r' + ' ' * 80 + '\r', end='')

            print(f'     {arrow_up}')
            print(f'    {arrow_left}{arrow_down}{arrow_right}\n\n')
            print(f"Velocity:    {data['actual_vel'][0]} [cm/s]       {data['actual_vel'][0]} [deg/s]")
            print(f"Wheels:     {data['actual_vel_wheels'][0]} [rpm]       {data['actual_vel_wheels'][1]} [rpm]")
            print("Position: ")
            print(f"    X: {data['position'][0]} [cm]       Y: {data['position'][1]} [cm]       θ: {data['position'][2]} [deg]\n\n")
            print(f"Loop time:  arduino {data['time_arduino_us']/1000}[ms]    rpi {data['time_rpi_ms']} [ms]")
            print("\n\n")
