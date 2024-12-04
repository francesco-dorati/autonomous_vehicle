import time
from raspberry_pi.drivers.nano import NANO

def main():
    NANOTester.test()


class NANOTester:
    @staticmethod
    def test():
        NANO.start()
        NANOTester.ping_test()
        NANOTester.battery_test()
        NANO.stop()


    @staticmethod
    def ping_test():
        t = time.time()
        ok = NANO.ping()
        dt = (time.time() - t)*1000
        if not ok:
            print("NO PING RETURN")
            return
        print(f"Ping: OK,   took: {dt} ms")

    @staticmethod
    def battery_test():
        t = time.time()
        battery = NANO.get_battery()
        dt = (time.time() - t)*1000
        print(f"Battery: {battery/1000} V,  took: {dt} ms")
        
if __name__ == "__main__":
    main()