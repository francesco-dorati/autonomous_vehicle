import time
from raspberry_pi.robot import Robot

def main():
    robot = Robot()
    robot.start()
    robot.start_mapping()

    robot.set_target_velocity(300, 0)
    robot.save_global_map("map_start")
    time.sleep(5)

    robot.stop_control()
    robot.save_global_map("map_end")

    robot.stop_mapping()
    robot.stop()



if __name__ == "__main__":
    main()
