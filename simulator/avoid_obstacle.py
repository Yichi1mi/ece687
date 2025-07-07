from mobile_manipulator_unicycle_sim import MobileManipulatorUnicycleSim
from dwa_planner import dwa_planner
import time
import math

def avoid_obstacles(robot: MobileManipulatorUnicycleSim):
    reached_pickup = False
    straight_mode = False

    while True:
        robot_pose, pickup, dropoff, _, _ = robot.get_poses()
        goal = pickup if not reached_pickup else dropoff

        gripper_x = robot_pose[0] + 0.35 * math.cos(robot_pose[2])
        gripper_y = robot_pose[1] + 0.35 * math.sin(robot_pose[2])
        distance = math.hypot(gripper_x - goal[0], gripper_y - goal[1])
        
        robot_x, robot_y, robot_theta = robot_pose
        target_angle = math.atan2(goal[1] - robot_y, goal[0] - robot_x)
        heading_error = (target_angle - robot_theta + math.pi) % (2 * math.pi) - math.pi
        heading_error_deg = math.degrees(heading_error)

        if not straight_mode:
            if distance < 0.35 and abs(heading_error_deg) < 8:
                print("Entering straight mode")
                straight_mode = True
            else:
                dwa_planner(robot, goal)
        else:
            forward_speed = 0.05
            small_w = math.radians(max(min(heading_error_deg, 5), -5))
            robot.set_mobile_base_speed_and_gripper_power(forward_speed, small_w, 0)
            time.sleep(0.05)
            if distance < 0.3 and abs(heading_error_deg) > 15:
                print("Lost alignment in straight mode. Returning to DWA.")
                start_time = time.time()
                while time.time() - start_time < 3.5:
                    robot.set_mobile_base_speed_and_gripper_power(-0.1, 0, 0)
                    time.sleep(0.05)
                straight_mode = False
            if distance < 0.03:
                robot.set_mobile_base_speed_and_gripper_power(0, 0, 0)
                print("Reached!")
                time.sleep(1)
                start_time = time.time()
                while time.time() - start_time < 2.:
                    robot.set_mobile_base_speed_and_gripper_power(-0.1, 0, 0)
                    time.sleep(0.05)
                if not reached_pickup:
                    reached_pickup = True
                    straight_mode = False
                else:
                    break