import time
import matplotlib.pyplot as plt
import numpy as np

from raspberry_pi.devices.lidar import Lidar
from raspberry_pi.data_structures.maps import LocalMap, GlobalMap
from raspberry_pi.data_structures.states import Position
from raspberry_pi.devices.rp2040 import RP2040
def main():
    # LidarTester.stop_scan()
    # return
    # LidarTester.test_global_map()
    LidarTester.test_static_dist()

    # LidarTester.test_local_maps()
  

class LidarTester:
    @staticmethod
    def stop_scan():
        Lidar.start()
        Lidar.stop_scan()
        Lidar.stop()
        
    @staticmethod
    def test_local_maps():
        Lidar.start()
        Lidar.ping()
        Lidar.start_scan()
        time.sleep(20)

        i = 0
        maps = []
        t_start = time.time()
        while (time.time() - t_start) < 10:
            local_map = Lidar.produce_local_map()
            maps.append(local_map)
            i += 1
            time.sleep(1)

        Lidar.stop_scan()
        Lidar.stop()

        for i, local_map in enumerate(maps):
            local_map.draw(f"lidar{i}")  

    @staticmethod
    def test_global_map():
        Lidar.start()
        Lidar.ping()
        Lidar.start_scan()

        global_map = GlobalMap()
        pos = Position(0, 0, 0)
        time.sleep(3)

        local_map = Lidar.produce_local_map()

        Lidar.stop_scan()
        Lidar.stop()

        local_map.draw("local_map")
        global_map.expand(pos, local_map)
        global_map.draw("global_map")

    @staticmethod
    def test_static_dist():
        RP2040.start()
        RP2040.reset_position()
        pos0 = Position(0, 0, 0)
        pos1 = Position(2500, 0, 0)
        pos2 = Position(2500, 1500, 1700)
        global_map = GlobalMap()

        Lidar.start()
        Lidar.start_scan()
        time.sleep(4)

        l0 = Lidar.produce_local_map()
        print(Lidar._scanning)
        global_map.expand(pos0, l0)

        RP2040.follow_path([pos1])
        time.sleep(10)

        l1 = Lidar.produce_local_map()
        global_map.expand(pos1, l1)

        RP2040.follow_path([pos2])
        time.sleep(10)

        l2 = Lidar.produce_local_map()
        global_map.expand(pos2, l2)

        Lidar.stop_scan()
        Lidar.stop()
        l0.draw("local0")
        l1.draw("local1")
        l2.draw("local2")
        global_map.draw("global")
        RP2040.stop()
        
    # @staticmethod
    # def _draw_scan(scan, name="test"):
    #     path_name = f"../img/{name}.png"
    #     x_points = []
    #     y_points = []
        
    #     for angle, distance in enumerate(scan):
    #         # Convert angle from degrees to radians for trigonometric functions
    #         angle_rad = np.radians(360 - angle)
            
    #         # Skip points with invalid or zero distance
    #         if distance > 0:
    #             x = distance * np.cos(angle_rad)
    #             y = distance * np.sin(angle_rad)
    #             x_points.append(x)
    #             y_points.append(y)
                
    #     # Plotting
    #     plt.figure(figsize=(8, 8))
    #     plt.scatter(x_points, y_points, s=1, color='blue')  # Smaller marker for a denser plot

    #     # Robot position
    #     plt.plot(0, 0, 'ro', label="Robot Position")  # Red dot for robot location
    #     plt.arrow(0, 0, 500, 0, head_width=100, head_length=80, fc='red', ec='red', label="Heading")
    #     plt.gca().invert_yaxis()  # Invert y-axis to make the plot look correct

    #     plt.title("LIDAR Scan")
    #     plt.xlabel("X (mm)")
    #     plt.ylabel("Y (mm)")
    #     plt.axis('equal')  # Ensures aspect ratio is equal for X and Y axes
    #     plt.grid(True)
    #     plt.savefig(path_name, format='png')
    #     plt.close() 

if __name__ == "__main__":
    main()
    

        
