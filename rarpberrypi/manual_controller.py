import time

MANUAL_LOOP_FREQ = 50 # Hz
MANUAL_TAO = (1/MANUAL_LOOP_FREQ) # s

MANUAL_LIN_VEL = 18 # cm/s
MANUAL_ANG_VEL = 40 # deg/s


class ManualController:
    def __init__(self, socket, serial):
        self.socket = socket
        self.serial = serial

        self.serial.start()

    def loop(self):
        skip = 0
        lin_vel, ang_vel = (0, 0)
        while self.socket.connected:
            t_start = time.time()
            self.serial.read()

            if not self.socket.queue.empty():
                skip = 0
                lin_vel, ang_vel = (0, 0)
                keyboard_buffer = self.socket.queue.get()

                if "f" in keyboard_buffer:
                    lin_vel = MANUAL_LIN_VEL
                if "b" in keyboard_buffer:
                    lin_vel = -MANUAL_LIN_VEL
                if "l" in keyboard_buffer:
                    ang_vel = MANUAL_ANG_VEL
                if "r" in keyboard_buffer:
                    ang_vel = -MANUAL_ANG_VEL

                # SEND LOG BACK TO SOCKET
                # self.socket.send(data)

            else:
                skip += 1
                if skip > MANUAL_LOOP_FREQ*1:
                    lin_vel, ang_vel = (0, 0)
                    skip = 0

            self.serial.send(lin_vel, ang_vel)

            dt = time.time() - t_start
            if dt < MANUAL_TAO:
                time.sleep(MANUAL_TAO - dt)

    def end_connection(self):
        self.socket.end_connection()
