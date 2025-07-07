from mobile_manipulator_unicycle import MobileManipulatorUnicycle
import time

robot = MobileManipulatorUnicycle(robot_id=1, backend_server_ip="192.168.0.2")

print("Move forward for 2 seconds and set the LEDs red")
start_time = time.time()
while time.time() - start_time < 2.:
    robot.set_mobile_base_speed_and_gripper_power(v=0.5, omega=0.0, gripper_power=0.0)
    robot.set_leds(255, 0, 0)
    time.sleep(0.05)

print("Move backward for 2 seconds and set the LEDs red")
start_time = time.time()
while time.time() - start_time < 2.:
    robot.set_mobile_base_speed_and_gripper_power(v=-0.5, omega=0.0, gripper_power=0.0)
    robot.set_leds(255, 0, 0)
    time.sleep(0.05)

print("Rotate CCW for 2 seconds and set the LEDs green")
start_time = time.time()
while time.time() - start_time < 2.:
    robot.set_mobile_base_speed_and_gripper_power(v=0.0, omega=10.0, gripper_power=0.0)
    robot.set_leds(0, 255, 0)
    time.sleep(0.05)

print("Rotate CW for 2 seconds and set the LEDs green")
start_time = time.time()
while time.time() - start_time < 2.:
    robot.set_mobile_base_speed_and_gripper_power(v=0.0, omega=-10.0, gripper_power=0.0)
    robot.set_leds(0, 255, 0)
    time.sleep(0.05)

print("Move arm")
robot.set_arm_pose(100, 0)
robot.set_arm_pose(0, 0)
robot.set_arm_pose(0, 100)
robot.set_arm_pose(0, 0)

print("Close the gripper for 2 seconds and set the LEDs blue")
start_time = time.time()
while time.time() - start_time < 2.:
    robot.set_mobile_base_speed_and_gripper_power(v=0.0, omega=0.0, gripper_power=-1.0)
    robot.set_leds(0, 0, 255)
    time.sleep(0.05)

print("Open the gripper for 2 seconds and set the LEDs blue")
start_time = time.time()
while time.time() - start_time < 2.:
    robot.set_mobile_base_speed_and_gripper_power(v=0.0, omega=0.0, gripper_power=1.0)
    robot.set_leds(0, 0, 255)
    time.sleep(0.05)

print("Stop the base and the gripper")
robot.set_mobile_base_speed_and_gripper_power(0., 0., 0.)

print("Get the robot's current pose")
poses = robot.get_poses()
print(f"Robot, pickup, dropoff, obstacles poses: {poses}")