from lidar import Lidar
import time

l = Lidar()
l.start_scan()

time.sleep(15)
l.stop_scan()

        