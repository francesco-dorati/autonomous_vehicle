from raspberry_pi.drivers.lidar import Lidar
import time
import matplotlib.pyplot as plt
import numpy as np

def main():
    Lidar.start()
    Lidar.get_health()
    time.sleep(0.5)
    Lidar.start_scan()
    time.sleep(10)
    Lidar.stop_scan()
    Lidar.stop()


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

def draw_scan(scan, name="test"):
    path_name = f"../../img/{name}.png"
    # Convert polar coordinates to Cartesian
    x_points = []
    y_points = []
    
    for angle, distance, _ in scan:
        # Convert angle from degrees to radians for trigonometric functions
        angle_rad = np.radians(angle)
        
        # Skip points with invalid or zero distance
        if distance > 0:
            x = distance * np.cos(angle_rad)
            y = distance * np.sin(angle_rad)
            x_points.append(x)
            y_points.append(y)
            
    # Plotting
    plt.figure(figsize=(8, 8))
    plt.scatter(x_points, y_points, s=1, color='blue')  # Smaller marker for a denser plot

    # Robot position
    plt.plot(0, 0, 'ro', label="Robot Position")  # Red dot for robot location
    plt.arrow(0, 0, 500, 0, head_width=100, head_length=80, fc='red', ec='red', label="Heading")
    plt.gca().invert_yaxis()  # Invert y-axis to make the plot look correct

    plt.title("LIDAR Scan")
    plt.xlabel("X (mm)")
    plt.ylabel("Y (mm)")
    plt.axis('equal')  # Ensures aspect ratio is equal for X and Y axes
    plt.grid(True)
    plt.savefig(path_name, format='png')
    plt.close()

if __name__ == "__main__":
    main()
    
# l = Lidar()
# l.start_scan()


# time.sleep(5)
# l1 = l.get_scan()
# print("scan 1")

# time.sleep(1)
# l2 = l.get_scan()
# print("scan 2")

# time.sleep(1)
# l3 = l.get_scan()
# print("scan 3")

# time.sleep(1)
# l4 = l.get_scan()
# print("scan 4")

# time.sleep(1)
# l5 = l.get_scan()
# print("scan 5")

# l.stop_scan()

# print_scan(l1)
# print_scan(l2)
# print_scan(l3)
# draw_scan(l1, "test1")
# draw_scan(l2, "test2")
# draw_scan(l3, "test3")
# draw_scan(l4, "test4")
# draw_scan(l5, "test5")

        
