from mobile_manipulator_unicycle_sim import MobileManipulatorUnicycleSim
from avoid_obstacle_sim import avoid_obstacles
import time

# Two options to initialize the simulator
# (i) random robot and object locations
robot = MobileManipulatorUnicycleSim(robot_id=1)
# (ii) specified robot and object locations
# robot = MobileManipulatorUnicycleSim(robot_id=1, robot_pose=[0.0, 0.0, 0.0], pickup_location=[0.75, 0.75], dropoff_location=[-0.75, -0.75], obstacles_location=[[0.5, 0.5], [-0.5, -0.5]])

# print("Move forward for 2 seconds")
# start_time = time.time()
# while time.time() - start_time < 2.:
#     robot.set_mobile_base_speed_and_gripper_power(v=0.5, omega=0.0, gripper_power=0.0)
#     time.sleep(0.05)

# print("Move backward for 2 seconds")
# start_time = time.time()
# while time.time() - start_time < 2.:
#     robot.set_mobile_base_speed_and_gripper_power(v=-0.5, omega=0.0, gripper_power=0.0)
#     time.sleep(0.05)

# print("Rotate CCW for 2 seconds")
# start_time = time.time()
# while time.time() - start_time < 2.:
#     robot.set_mobile_base_speed_and_gripper_power(v=0.0, omega=10.0, gripper_power=0.0)
#     time.sleep(0.05)

# print("Rotate CW for 2 seconds")
# start_time = time.time()
# while time.time() - start_time < 2.:
#     robot.set_mobile_base_speed_and_gripper_power(v=0.0, omega=-10.0, gripper_power=0.0)
#     time.sleep(0.05)

# print("Stop the base and the gripper")
# robot.set_mobile_base_speed_and_gripper_power(0., 0., 0.)

# print("Get the robot's current pose")
# poses = robot.get_poses()
# print(f"Robot, pickup, dropoff, obstacles poses: {poses}")
avoid_obstacles(robot)