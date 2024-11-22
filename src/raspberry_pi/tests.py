import time

def main():
    p = PowerTest()
    p.test_1()




class PowerTest:
    """
        - Test if motors are correctly connected
        - Test if there is drifting
    """
    def __init__(self):
        import time
        from rp2040 import RP2040
        self.rp2040 = RP2040()
    
    def test_1(self):
        try:
            print("TEST ONE")
            print("DRIFTING AND DIRECTIONS")

            print("STRAIGHT")
            self.straight(3)
            print("END STRAIGHT")
            time.sleep(1)

            print("BACK")
            self.backwards(3)
            print("END BACK")
            time.sleep(1)

            print("LEFT")
            self.rotate_left(3)
            print("END LEFT")
            time.sleep(1)
            
            print("RIGHT")
            self.rotate_right(3)
            print("END RIGHT")
            time.sleep(1)

            print("Check if in starting position")

        except:
            self.rp2040.stop_motors()
    
    def test_2(self):
        try:
            while True:
                c = int(input("Input power (0, 255): "))
                if c == -1:
                    break
                if c > 0 and c < 255:
                    self.rp2040.set_target_power(c, c)
                    time.sleep(3)
                    _, _, _, vl, _ = self.rp2040.request_odometry()
                    self.rp2040.stop_motors()
                    print(f"Velocity: {vl} mm/s")

        except:
            self.rp2040.stop_motors()

    def test_power(self, l, r, time):
        self.rp2040.set_target_power(l, r)
        time.sleep(time)
        self.rp2040.stop_motors()

    def straight(self, time):
        self.test_power(100, 100, time)
    
    def backwards(self, time):
        self.test_power(-100, -100, time)

    def rotate_left(self, time):
        self.test_power(-100, 100, time)

    def rotate_right(self, time):
        self.test_power(100, -100, time)





if __name__ == "__main__":
    main()
