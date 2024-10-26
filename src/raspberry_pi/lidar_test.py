from lidar import Lidar
import time

l = Lidar()
l.start_scan()

time.sleep(5)
print(l.get_scan())
time.sleep(5)
print(l.get_scan())
time.sleep(5)
print(l.get_scan())
time.sleep(5)
print(l.get_scan())
l.stop_scan()

        