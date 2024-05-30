import queue
import time
import math

AUTO_LOOP_FREQ = 50 # Hz
AUTO_TAO = (1/AUTO_LOOP_FREQ) # s
AUTO_LIN_VEL = 20 # cm/s
AUTO_ANG_VEL = 60 # deg/s
AUTO_LIN_ERROR = 2 # cm
AUTO_ANG_ERROR = 5 # deg

TICK_PER_REV = 0
WHEEL_DIST_FROM_CENTER = 12

class state:
    def __init__(self, x = .0, y = .0, t = .0):
        self.x = x
        self.y = y
        self.theta = t
        

class AutonomousController:
    def __init__(self, socket, serial):
        self.socket = socket
        self.serial = serial
        self.goal_state = state()
        self.current_state = state()
        self.states_queue = queue.Queue()
        self.goal_reached = False

    def loop(self):
        while True:
            t_start = time.time()
            goal_reached = False
            
            # read command from socket
            if not self.socket.queue.empty():
                command = self.socket.queue.get()
                if command == "EXIT":
                    self._reset()
                    return
                self._add_command(command)

            # load new command
            if self.goal_reached:
                if not self.states_queue.empty():
                    # update goal state
                    self.goal_state = self.states_queue.get()
                    self.goal_reached = False

            # receive odometry data
            data = self.serial.read()
            tick_right, tick_left, dist, log = self._read_data(data)
            self._update_odometry(tick_right, tick_left)

            # get new state 
            while True:
                rho, alpha, beta = self._calculate_position_error()

                rho_ok = abs(rho) < AUTO_LIN_ERROR
                alpha_ok = abs(alpha) < AUTO_ANG_ERROR
                beta_ok = abs(beta) < AUTO_ANG_ERROR

                if rho_ok and alpha_ok and beta_ok:
                    if not self.states_queue.empty():
                        self.goal_state = self.states_queue.get()
                        goal_reached = True
                    else:
                        rho = 0
                        alpha = 0
                        beta = 0
                else:
                    break
                

            # calculate goal velocity
            lin_vel, ang_vel = self._position_controller(rho, alpha, beta, dist)

            # send to serial
            self.serial.send(lin_vel, ang_vel)
           
            # send respose to dev
            data = self._produce_response(rho, alpha, beta, lin_vel, ang_vel, dist, log)
            self.socket.send(data)

            dt = time.time() - t_start
            if AUTO_TAO < dt:
                time.sleep(AUTO_TAO - dt)
            

    def _add_command(self, command):
        try:
            c = command.split()
            if c[0] == "mv":
                self.commands_queue.put((int(c[1]), .0, .0))
                return
            
            elif c[0] == "rot":
                self.commands_queue.put((.0, .0, int(c[1])))
                return
        except:
            raise Exception(f"Invalid command: '{command}'")
    
    def _read_data(self, data):
        pass

    def _update_odometry(self, tick_r, tick_l):
        dist_r = tick_r*2*math.pi / TICK_PER_REV
        dist_l = tick_l*2*math.pi / TICK_PER_REV

        dt = (dist_r - dist_l) / WHEEL_DIST_FROM_CENTER
        if dt == 0:
            dx = dist_l*math.cos(self.current_state.theta)
            dy = dist_l*math.sin(self.current_state.theta)
        else:
            dx = (dist_l+dist_l)*math.cos(self.current_state.theta + dt/2) / 2
            dx = (dist_l+dist_l)*math.sin(self.current_state.theta + dt/2) / 2

        self.current_state[0] += dx
        self.current_state[1] += dy
        self.current_state[2] += dt

    def _calculate_position_error(self):
        dx = self.goal_state.x - self.current_state.x
        dy = self.goal_state.y - self.current_state.y
        dt = self.goal_state.theta - self.current_state.theta

        rho = math.sqrt(dx*dx + dy*dy)

        alpha = math.atan2(dy, dx)*(180/math.pi) - self.current_state.theta
        alpha = (alpha + math.pi) % 2*math.pi - math.pi

        beta = dt - alpha
        beta = (beta + math.pi) % 2*math.pi - math.pi

        return rho, alpha, beta


    def _position_controller(self, rho, alpha, beta):
        pass
        # dx = self.goal_position[0] - self.current_state[0]
        # dy = self.goal_position[1] - self.current_state[1]
        # dt = self.goal_position[2] - self.current_state[2]
        # while abs(dx) < AUTO_LIN_ERROR and abs(y) < AUTO_LIN_ERROR and abs(dt) < AUTO_ANG_ERROR:
        #     # GOAL REACHED
        #     # check new command
        #     if self.movements_queue.empty():
        #         return (.0, .0)
            
        #     dx, dy, dt = self.movements_queue.get()
        #     self.goal_position[0] += dx
        #     self.goal_position[1] += dy
        #     self.goal_position[2] += dt

        # # calculate velocity
        # if dx > 0:
        #     lin_vel = AUTO_LIN_VEL
        # elif dx < 0:
        #     lin_vel = -AUTO_LIN_VEL
        
        # if dt > 0:
        #     ang_vel = AUTO_ANG_VEL
        # elif dt < 0:
        #     ang_vel = -AUTO_ANG_VEL

        # return (lin_vel, ang_vel)

    def _produce_response(self, rho, alpha, beta, lin_vel, ang_vel, dist, log):
        pass

    def _reset(self):
        self.current_state = (.0, .0, .0)
        self.movements_queue = queue.Queue()
        