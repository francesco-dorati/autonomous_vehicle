import time
import math
import json

MANUAL_LOOP_FREQ = 50 # Hz
MANUAL_TAO = (1/MANUAL_LOOP_FREQ) # s

MANUAL_LIN_VEL = 20 # cm/s
MANUAL_ANG_VEL = 50 # deg/s

WHEEL_RADIUS = 3.4
DIST_FROM_CENTER = 12
TICKS_PER_REV = 1495


class ManualController:
    def __init__(self, socket, serial):
        self.current_state = [.0, .0, .0]

        self.socket = socket
        self.serial = serial

        self.serial.start()




    def loop(self):
        skip = 0
        lin_vel, ang_vel = (0, 0)
        while self.socket.connected:
            t_start = time.time()
            received = False

            if not self.socket.queue.empty():
                skip = 0
                lin_vel, ang_vel = (0, 0)
                keyboard_buffer = self.socket.queue.get()
                received = True

                if "f" in keyboard_buffer:
                    lin_vel = MANUAL_LIN_VEL
                elif "F" in keyboard_buffer:
                    lin_vel = MANUAL_LIN_VEL*1.5

                if "b" in keyboard_buffer:
                    lin_vel = -MANUAL_LIN_VEL
                elif "B" in keyboard_buffer:
                    lin_vel = -MANUAL_LIN_VEL*1.5

                if "l" in keyboard_buffer:
                    ang_vel = MANUAL_ANG_VEL
                elif "L" in keyboard_buffer:
                    ang_vel = MANUAL_ANG_VEL*1.5

                if "r" in keyboard_buffer:
                    ang_vel = -MANUAL_ANG_VEL
                elif "R" in keyboard_buffer:
                    ang_vel = -MANUAL_ANG_VEL*1.5
            
            self.serial.send(lin_vel, ang_vel)
            s = self.serial.read()

            if received:
                data = self.process_data(s, t_start, lin_vel, ang_vel)
                self.socket.send(json.dumps(data))

            dt = time.time() - t_start
            print(f"[MANUAL] Loop time: {(dt*1000):.3f} ms instead of {(MANUAL_TAO*1000)} ms")
            if dt < MANUAL_TAO:
                time.sleep(MANUAL_TAO - dt)

    def process_data(self, data_string, t_start, lin_vel, ang_vel):
        
        left, right, t, _ = data_string.split(";")
        left = left.split(" ")
        right = right.split(" ")
        t = t.split(" ")

        print(left, right, t)

        data = {}
        data["goal_vel_wheels"] = [float(left[1]), float(right[1])]
        data["actual_vel_wheels"] = [float(left[2]), float(right[2])]
        data["power_motors"] = [float(left[3]), float(right[3])]
        data["encoder_ticks"] = [int(left[4]), int(right[4])]

        data["goal_vel"] = [lin_vel, ang_vel]
        data["actual_vel"] = self._kinematics(data["actual_vel_wheels"])
        data["position"] = self._calculate_odometry(data["encoder_ticks"])

        data["time_arduino_us"] = float(t[1])
        data["time_rpi_ms"] = (time.time() - t_start)*1000
        return data
    
    def _kinematics(self, w_vel):
        # calcola velocità delle ruote
        vx = (w_vel[0] + w_vel[1])*math.pi*WHEEL_RADIUS / 30
        va = (w_vel[1] - w_vel[0])*3*WHEEL_RADIUS / DIST_FROM_CENTER
        return [vx, va]

    def  _calculate_odometry(self, ticks):
        space_l = (ticks[0]*2*math.pi*WHEEL_RADIUS) / TICKS_PER_REV
        space_r = (ticks[1]*2*math.pi*WHEEL_RADIUS) / TICKS_PER_REV
        dtheta = (space_r - space_l) / DIST_FROM_CENTER
        dx = (space_l + space_r) * math.cos(self.current_state[2] + dtheta) / 2
        dy = (space_l + space_r) * math.sin(self.current_state[2] + dtheta) / 2

        self.current_state[0] += dx
        self.current_state[1] += dy
        self.current_state[2] += dtheta

        return self.current_state
        





