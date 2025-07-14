from mobile_manipulator_unicycle import MobileManipulatorUnicycle
from controller import run_pick_and_place_mission
import time

COLOR_WHITE = (255, 255, 255)

robot = MobileManipulatorUnicycle(robot_id=1, backend_server_ip="192.168.0.2")

# Wait for all poses to be available to increase stability
print("Waiting for all pose data to be ready...")
while not all(robot.get_poses()):
    robot.set_leds(*COLOR_WHITE)
    time.sleep(0.5)
print("Pose data is ready. Starting the mission!")

run_pick_and_place_mission(robot)

robot.set_leds(0, 0, 0)
print("Program has exited.")