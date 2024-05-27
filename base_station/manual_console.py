import time
import keyboard
import json
import socket

MANUAL_FREQ = 10 # Hz
MANUAL_TAO = (1/MANUAL_FREQ) # s

class ManualConsole:
    def __init__(self, hostname, port):
        self.hostname = hostname
        self.port = port
        print("\n[MANUAL] Starting manual controller...")
        print(f"[MANUAL] Connecting to {self.hostname}:{self.port}...")
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.socket.sendto("SYN\n".encode('utf-8'), (self.hostname, self.port))
            print("[MANUAL] SYN sent.")
            ack, _ = self.socket.recvfrom(1024)
            print("[MANUAL] response.")
            if ack.decode().strip() == "ACK":
                print("[MANUAL] Connection established.")
            else:
                raise Exception(f"[MANUAL] Connection failed. Received '{ack}'.")
            
        except socket.error:
            raise Exception("[MANUAL] Connection failed.")
        
        print("\nMANUAL CONSOLE\n")
        print("         W: Forward")
        print("A: Left  S: Backward  D: Right\n")
        print("(esc) or (space) to exit.")
        print("\n\n\n\n")

    def run(self):
        try:
            while True:
                start_time = time.time()
                input_buffer = []

                if keyboard.is_pressed('esc') or keyboard.is_pressed("space"):
                    self.socket.sendto("EXIT".encode('utf-8'), (self.hostname, self.port_manual))
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
                
                self._print_direction(input_buffer)

                # send buffer
                self.socket.sendto(''.join(input_buffer).encode('utf-8'), (self.hostname, self.port))

                # wait until next iteration
                dt = time.time() - start_time
                if dt < MANUAL_TAO:
                    time.sleep(MANUAL_TAO - dt) 
        except:
            self.socket.close()
            return

    def _print_direction(self, input_buffer):
            arrow_up = '\u2191' if "f" in input_buffer else ''
            arrow_down = '\u2193' if "b" in input_buffer else ''
            arrow_left = '\u2190' if "l" in input_buffer else ' '
            arrow_right = '\u2192' if "r" in input_buffer else ' '

            print('\r' + ' ' * 80 + '\r ', end='')
            print('\x1b[A' + '\r' + ' ' * 80 + '\r', end='')
            print('\x1b[A' + '\r' + ' ' * 80 + '\r', end='')
            print('\x1b[A' + '\r' + ' ' * 80 + '\r', end='')
            print(f'    {arrow_left}{arrow_up}{arrow_down}{arrow_right}\n\n')