from mobile_manipulator_unicycle import MobileManipulatorUnicycle
from main_controller import run_pick_and_place_mission, initialize_robot
import time
from config import *


robot = MobileManipulatorUnicycle(robot_id=1, backend_server_ip="192.168.0.2")

initialize_robot(robot)

# Wait for all poses to be available to increase stability
print("Waiting for all pose data to be ready...")
while not all(robot.get_poses()):
    robot.set_leds(*COLOR_WHITE)
    time.sleep(1.0)
print("Pose data is ready. Starting the mission!")

run_pick_and_place_mission(robot)

robot.set_leds(*COLOR_WHITE)
print("Program has exited.")