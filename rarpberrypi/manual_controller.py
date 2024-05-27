import time

MANUAL_LOOP_FREQ = 10 # Hz
MANUAL_TAO = (1/MANUAL_LOOP_FREQ) # s

MANUAL_LIN_VEL = 18 # cm/s
MANUAL_ANG_VEL = 40 # deg/s


class ManualController:
    def __init__(self, socket, serial):
        self.socket = socket
        self.serial = serial

    def loop(self):
        while self.socket.connected:
            t_start = time.time()
            lin_vel, ang_vel = (.0, .0)
            self.serial.read()

            if not self.socket.queue.empty():
                keyboard_buffer = self.socket.queue.get()

                if "f" in keyboard_buffer:
                    lin_vel += MANUAL_LIN_VEL
                if "b" in keyboard_buffer:
                    lin_vel -= MANUAL_LIN_VEL
                if "l" in keyboard_buffer:
                    ang_vel += MANUAL_ANG_VEL
                if "r" in keyboard_buffer:
                    ang_vel -= MANUAL_ANG_VEL

            self.serial.send(lin_vel, ang_vel)

            dt = time.time() - t_start
            if dt < MANUAL_TAO:
                time.sleep(MANUAL_TAO - dt)

    def end_connection(self):
        self.socket.end_connection()
