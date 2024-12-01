import time
import matplotlib.pyplot as plt
from raspberry_pi.drivers.rp2040 import RP2040


def main():
    PositionTester.test_static()

def stop():
    RP2040.stop_motors()

class Tester:
    @staticmethod
    def set_power(l, r, time):
        RP2040.set_target_power(l, r)
        time.sleep(time)
        RP2040.stop_motors()
    



class PowerTester(Tester):
    @staticmethod
    def test_motors():
        """ Test correct directions """
        try:
            print("TEST ONE")
            print("DRIFTING AND DIRECTIONS")

            print("STRAIGHT")
            PowerTester.straight(3)
            print("END STRAIGHT")
            time.sleep(1)

            print("BACK")
            PowerTester.backwards(3)
            print("END BACK")
            time.sleep(1)

            print("LEFT")
            PowerTester.rotate_left(3)
            print("END LEFT")
            time.sleep(1)
            
            print("RIGHT")
            PowerTester.rotate_right(3)
            print("END RIGHT")

            print("Check if in starting position")

        except _ as e:
            print(e)
            RP2040.stop_motors()

    @staticmethod
    def test_velocity():
        """ Get velocity value based on power """
        try:
            while True:
                print("Input power (0, 255): ")
                l = int(input('[left] '))
                r = int(input('[right] '))
                if (l > 0 and l < 255) and (r > 0 and r < 255):
                    RP2040.set_target_power(l, r)
                    time.sleep(3)
                    _, _, _, vl, va = RP2040.get_debug_odometry()
                    RP2040.stop_motors()
                    print(f"Velocity: {vl} {va} mm/s")

        except _ as e:
            print(e)
            RP2040.stop_motors()

    @staticmethod
    def straight(time):
        Tester.set_power(100, 100, time)
    
    @staticmethod
    def backwards(time):
        Tester.set_power(-100, -100, time)

    @staticmethod
    def rotate_left(time):
        Tester.set_power(-100, 100, time)
    
    @staticmethod
    def rotate_right(time):
        Tester.set_power(100, -100, time)


class PositionTester(Tester):
    @staticmethod
    def test_time_position():
        """ Test the time it takes to get position """
        t_start = time.time()
        RP2040.get_position()
        dt = (time.time() - t_start)*1000
        print(f"Get Position takes {dt} ms") 

    @staticmethod
    def test_time_debug():
        """ Test the time it takes to get position """
        t_start = time.time()
        RP2040.get_debug_odometry()
        dt = (time.time() - t_start)*1000
        print(f"Get debug odom takes {dt} ms") 

    @staticmethod
    def test_static():
        """ Test by moving the wheels """
        RP2040.reset_position()
        while True:
            p = RP2040.get_position()
            print(p)
    
    @staticmethod
    def test_dynamic():
        """ Test by returning to origin """
        RP2040.reset_position()
        print("START")
        print(RP2040.get_position())
        Tester.set_power(100, 100, 3)
        print(RP2040.get_position())
        Tester.set_power(-100, -100, 3)
        print(RP2040.get_position())
        print("END")


class PidTester(Tester):
    @staticmethod
    def linear_static_test():
        kp = 0
        kd = 0
        ki = 0
        while True:
            i = input('[kp] ')
            if i:
                kp = i

            i = input('[kd] ')
            if i:
                kd = i

            i = input('[ki] ')
            if i:
                ki = i

            vel = input('[vel] ')

            print(kp, kd, ki, vel)
            PidTester.linear_test(kp, kd, ki, vel)
            

    @staticmethod
    def linear_test(kp, kd, ki, vl):
        front = PidTester.base_test(kp, kd, ki, vl, 0)
        print("FRONT", kp, kd, ki)
        PidTester.plot_data(front)
        time.wait(0.5)

        back = PidTester.base_test(kp, kd, ki, -vl, 0)
        print("BACK", kp, kd, ki)
        PidTester.plot_data(back)



    @staticmethod
    def base_test(kp, kd, ki, vl, va):
        """ Test correct pid values """
        sampling_time = 0.1
        test_time = 3
        data = []

        RP2040.set_pid_values(kp, ki, kd)
        RP2040.set_target_velocity(vl, va)

        t0 = time.time()
        while time.time() - t0 < test_time:
            t = time.time() - t0
            _, _, _, vl, va = RP2040.get_debug_odometry()
            data.append((t, vl, va))
            time.sleep(sampling_time)

        RP2040.stop_motors()
        return data


    @staticmethod
    def plot_data(data, vl_goal, va_goal):
        t_arr, vx_arr, va_arr = zip(*data)

        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 10))

        ax1.plot(t_arr, vx_arr, label="vl", color='blue', linewidth=2)
        ax1.axline(y=vl_goal, color='red', linestyle="--", label="Target")
        ax1.title("Linear Velocity")
        ax1.xlabel("time (s)")
        ax1.ylabel("linear vel (mm/s)")
        ax1.grid(True, linestyle="--", alpha=0.6)
        ax1.legend(fontsize=12)

        ax2.plot(t_arr, va_arr, label="va", color='blue', linewidth=2)
        ax2.axline(y=va_goal, color='red', linestyle="--", label="Target")
        ax2.title("Linear Velocity")
        ax2.xlabel("time (s)")
        ax2.ylabel("angular vel (mm/s)")
        ax2.grid(True, linestyle="--", alpha=0.6)
        ax2.legend(fontsize=12)

        plt.tight_layout()
        plt.show()
    


if __name__ == "__main__":
    main()