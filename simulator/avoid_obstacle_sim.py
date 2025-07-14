import math
import time
from mobile_manipulator_unicycle_sim import MobileManipulatorUnicycleSim
from clf_cbf_qp_controller import clf_cbf_qp_controller

def avoid_obstacles(robot: MobileManipulatorUnicycleSim):
    """
    Main control loop for navigating the robot to pickup and dropoff locations.

    This loop is greatly simplified because the complexity is handled by the
    CLF-CBF-QP controller. The loop's only responsibilities are:
    1.  Selecting the current goal (pickup or dropoff).
    2.  Calling the controller to generate and send motor commands.
    3.  Checking if the goal has been reached and switching to the next one.
    """
    
    # State variable to track the current phase of the mission
    # False = navigating to pickup, True = navigating to dropoff
    is_navigating_to_dropoff = False
    
    # Distance threshold to consider a goal "reached"
    REACHED_THRESHOLD = 0.02 # meters
    
    # Back-up duration after reaching a goal
    BACKUP_TIME = 2.0 # seconds
    BACKUP_SPEED = -0.08 # m/s

    while True:
        # === 1. Get state and determine the current goal ===
        robot_pose, pickup_loc, dropoff_loc, _ = robot.get_poses()
        
        goal = dropoff_loc if is_navigating_to_dropoff else pickup_loc
        
        # Use the gripper position for distance check, as it's the point of interest
        gripper_x = robot_pose[0] + 0.4 * math.cos(robot_pose[2])
        gripper_y = robot_pose[1] + 0.4 * math.sin(robot_pose[2])
        distance_to_goal = math.hypot(gripper_x - goal[0], gripper_y - goal[1])

        # === 2. Check for Goal Completion ===
        if distance_to_goal < REACHED_THRESHOLD:
            print(f"Goal reached at {goal}!")
            robot.set_mobile_base_speed_and_gripper_power(0, 0, 0)
            time.sleep(1.0) # Pause to simulate gripping/releasing

            # Perform a simple backup maneuver
            print("Backing up...")
            start_time = time.time()
            while time.time() - start_time < BACKUP_TIME:
                robot.set_mobile_base_speed_and_gripper_power(BACKUP_SPEED, 0, 0)
                time.sleep(0.05)
            robot.set_mobile_base_speed_and_gripper_power(0, 0, 0)

            # Switch to the next phase of the mission
            if not is_navigating_to_dropoff:
                is_navigating_to_dropoff = True
                print("Proceeding to dropoff location...")
            else:
                print("Mission complete!")
                break # Exit the main while loop
            
            continue # Restart the loop for the new goal

        # === 3. Call the Controller ===
        # If the goal is not reached, call the QP controller to calculate the
        # next safe and optimal move.
        clf_cbf_qp_controller(robot, goal)
        