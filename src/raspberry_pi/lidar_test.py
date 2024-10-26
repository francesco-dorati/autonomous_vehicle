from lidar import Lidar
import time

def print_scan(scan):
    print("SCAN:")
    for i in range(0, len(scan)):
        angle = scan[i][0]
        distance = scan[i][1]
        n = scan[i][2]
        # print(scan[i])
        print(f"a: {angle:.1f}°,\t d: {int(distance)} mm,\t n: {n}")
    print("SCAN END")
    print(f"size: {len(scan)}")
    print("\n\n")

l = Lidar()
l.start_scan()

time.sleep(5)

time.sleep(5)
print_scan(l.get_scan())
time.sleep(5)
print_scan(l.get_scan())
time.sleep(5)
print_scan(l.get_scan())
l.stop_scan()
        
