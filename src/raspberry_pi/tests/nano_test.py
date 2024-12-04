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
        # time.sleep(5)
        # NANO.start_sensors()
        # time.sleep(3)
        # NANOTester.sensors_test()
        # NANO.stop_sensors()
        NANO.stop()


    @staticmethod
    def ping_test():
        t = time.time()
        ok = NANO.ping()
        dt = (time.time() - t)*1000
        if not ok:
            print("NO PING RETURN")
            return
        print(f"Ping: OK,       took: {int(dt)} ms")

    @staticmethod
    def battery_test():
        t = time.time()
        battery = NANO.get_battery()
        dt = (time.time() - t)*1000
        if not battery:
            print("NO BATTERY DATA")
            return
        print(f"Battery: {(battery/1000):.2f} V,    took: {int(dt)} ms")

    @staticmethod
    def sensors_test():
        time.sleep(0.5)
        t = time.time()
        data = NANO.request_sensors()
        dt = (time.time() - t)*1000
        if not data:
            print("NO SENSOR DATA")
            return
        print(f"Sensors: {data[0]}, {data[1]}, {data[2]}, {data[3]}  [mm],      took {int(dt)} ms")


        
if __name__ == "__main__":
    main()