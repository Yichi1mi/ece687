import math
import time
from mobile_manipulator_unicycle_sim import MobileManipulatorUnicycleSim
from clf_cbf_qp_controller import clf_cbf_qp_controller

def avoid_obstacles(robot: MobileManipulatorUnicycleSim):
    
    is_navigating_to_dropoff = False
    
    REACHED_THRESHOLD = 0.02
    
    BACKUP_TIME = 2.0
    BACKUP_SPEED = -0.08

    while True:
        
        robot_pose, pickup_loc, dropoff_loc, _ = robot.get_poses()
        goal = dropoff_loc if is_navigating_to_dropoff else pickup_loc
        
        gripper_x = robot_pose[0] + 0.4 * math.cos(robot_pose[2])
        gripper_y = robot_pose[1] + 0.4 * math.sin(robot_pose[2])
        distance_to_goal = math.hypot(gripper_x - goal[0], gripper_y - goal[1])

        if distance_to_goal < REACHED_THRESHOLD:
            print(f"Goal reached at {goal}!")
            robot.set_mobile_base_speed_and_gripper_power(0, 0, 0)
            time.sleep(1.0) 

            # backup
            print("Backing up...")
            start_time = time.time()
            while time.time() - start_time < BACKUP_TIME:
                robot.set_mobile_base_speed_and_gripper_power(BACKUP_SPEED, 0, 0)
                time.sleep(0.05)
            robot.set_mobile_base_speed_and_gripper_power(0, 0, 0)

            if not is_navigating_to_dropoff:
                is_navigating_to_dropoff = True
                print("Proceeding to dropoff location...")
            else:
                print("Mission complete!")
                break
            
            continue

        clf_cbf_qp_controller(robot, goal)
        