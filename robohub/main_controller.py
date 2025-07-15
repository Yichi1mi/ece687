import math
import time
import numpy as np
from mobile_manipulator_unicycle import MobileManipulatorUnicycle
from clf_cbf_qp_controller import clf_cbf_qp_controller
from config import *

def perform_pickup(robot: MobileManipulatorUnicycle):
    """
    Executes the pre-defined sequence for picking up an object.
    Uses open-loop time delays for arm and gripper actions.
    """
    print("Initiating pickup sequence...")
    robot.set_leds(*COLOR_YELLOW)

    print("  [Step 1/5] Moving arm to pre-grasp position")
    start_time = time.time()
    while time.time() - start_time < 1.6:
        robot.set_arm_pose(100, 50)
        time.sleep(0.05)

    print("  [Step 2/5] Opening gripper")
    start_time = time.time()
    while time.time() - start_time < 2.1:
        robot.set_mobile_base_speed_and_gripper_power(v=0.0, omega=0.0, gripper_power=1.0)
        time.sleep(0.05)

    print("  [Step 3/5] Lowering arm to grasp position")
    start_time = time.time()
    while time.time() - start_time < 1.6:
        robot.set_arm_pose(200, -60)
        time.sleep(0.05)

    print("  [Step 4/5] Closing gripper")
    start_time = time.time()
    while time.time() - start_time < 2.5:
        robot.set_mobile_base_speed_and_gripper_power(v=0.0, omega=0.0, gripper_power=-1.0)
        time.sleep(0.05)

    print("  [Step 5/5] Lifting object")
    start_time = time.time()
    while time.time() - start_time < 2.4:
        robot.set_arm_pose(100, 50)
        time.sleep(0.05)
        
    robot.set_mobile_base_speed_and_gripper_power(v=0.0, omega=0.0, gripper_power=0.0)
    print("Pickup sequence complete.")





def perform_dropoff(robot: MobileManipulatorUnicycle):
    """
    Executes the pre-defined sequence for dropping off an object.
    """
    print("Initiating robust dropoff sequence...")
    robot.set_leds(*COLOR_YELLOW)

    # Action 1: Lower arm to dropoff position
    print("  [Step 1/3] Lowering arm to dropoff position...")
    start_time = time.time()
    while time.time() - start_time < 2.4:
        robot.set_arm_pose(200, -60)
        time.sleep(0.05)

    # Action 2: Open gripper to release object
    print("  [Step 2/3] Opening gripper to release object...")
    start_time = time.time()
    while time.time() - start_time < 2.4:
        robot.set_mobile_base_speed_and_gripper_power(v=0.0, omega=0.0, gripper_power=1.0)
        time.sleep(0.05)

    # Action 3: Retract arm
    print("  [Step 3/3] Retracting arm...")
    start_time = time.time()
    while time.time() - start_time < 2.4:
        robot.set_arm_pose(100, 50)
        time.sleep(0.05)
    
    # Ensure the final command is to stop gripper power
    robot.set_mobile_base_speed_and_gripper_power(v=0.0, omega=0.0, gripper_power=0.0)
    print("Dropoff sequence complete.")


def initialize_robot(robot: MobileManipulatorUnicycle):
    """
    Performs a robust initialization sequence to ensure the robot
    starts in a known, safe state (arm at 100, 50 and gripper open).
    """
    print("--- Starting Robot Initialization ---")
    
    # Step 1: Move arm to the default safe position (100, 50).
    print("  Initializing arm position...")
    start_time = time.time()
    while time.time() - start_time < 2.4: # Duration for arm movement
        robot.set_arm_pose(100, 50)
        time.sleep(0.05)
        
    # Step 2: Ensure the gripper is open.
    print("  Initializing gripper (opening)...")
    start_time = time.time()
    while time.time() - start_time < 2.1: # Duration for opening gripper
        robot.set_mobile_base_speed_and_gripper_power(v=0.0, omega=0.0, gripper_power=1.0)
        time.sleep(0.05)
        
    # Step 3: Stop sending power to the gripper motor to be safe.
    robot.set_mobile_base_speed_and_gripper_power(v=0.0, omega=0.0, gripper_power=0.0)
    
    print("--- Robot Initialization Complete ---")
    



def run_pick_and_place_mission(robot: MobileManipulatorUnicycle):
    """
    Main mission control loop with a refined, multi-stage state machine.
    Uses CLF-CBF for high-speed navigation and a P-controller for precision docking.
    """
    # Define the possible states for our new state machine.
    mission_phase = "NAVIGATE"  # NAVIGATE, FINAL_APPROACH, MANIPULATE
    is_navigating_to_dropoff = False
    
    last_robot_pose = None
    print("Mission started. Waiting for initial pose data...")

    while True:
        # Get latest poses and perform data validation.
        current_robot_pose, current_pickup_loc, current_dropoff_loc, current_obstacle_poses = robot.get_poses()
        if not all([current_robot_pose, current_pickup_loc, current_dropoff_loc]):
            time.sleep(0.01)
            continue
        if current_robot_pose == last_robot_pose:
            time.sleep(0.005) 
            continue
        last_robot_pose = current_robot_pose
        
        # --- COORDINATE FRAME CORRECTION ---
        x_r, y_r, theta = current_robot_pose
        x_c = x_r + D_CENTER * math.cos(theta)
        y_c = y_r + D_CENTER * math.sin(theta)

        # Determine current goal.
        goal = current_dropoff_loc if is_navigating_to_dropoff else current_pickup_loc
        
        # CORRECTED: Calculate distance and angle based on the geometric center.
        distance_to_goal = math.hypot(x_c - goal[0], y_c - goal[1])
        
        # --- State Machine Logic ---
        if mission_phase == "NAVIGATE":
            robot.set_leds(*COLOR_BLUE)
            
            if distance_to_goal > FINAL_APPROACH_DISTANCE:
                clf_cbf_qp_controller(robot, goal, current_robot_pose, current_obstacle_poses)
            else:
                print(f"NAVIGATE -> FINAL_APPROACH. Distance: {distance_to_goal:.2f}m.")
                start_time = time.time()
                while time.time() - start_time < 0.2:
                    robot.set_mobile_base_speed_and_gripper_power(0, 0, 0)
                    time.sleep(0.05)
                mission_phase = "FINAL_APPROACH"
        
        elif mission_phase == "FINAL_APPROACH":
            robot.set_leds(*COLOR_YELLOW)
            
            # CORRECTED: Errors for the P-controller are based on the geometric center.
            distance_error = distance_to_goal - GRIPPER_DISTANCE
            target_angle = math.atan2(goal[1] - y_c, goal[0] - x_c)
            angle_error = math.atan2(math.sin(target_angle - theta), math.cos(target_angle - theta))

            if abs(distance_error) < FINAL_DISTANCE_TOLERANCE and abs(angle_error) < FINAL_ANGLE_TOLERANCE:
                print(f"FINAL_APPROACH -> MANIPULATE. Pose aligned.")
                start_time = time.time()
                while time.time() - start_time < 0.2:
                    robot.set_mobile_base_speed_and_gripper_power(0, 0, 0)
                    time.sleep(0.05)
                mission_phase = "MANIPULATE"
            else:
                v = KP_LINEAR * distance_error
                omega = KP_ANGULAR * angle_error
                v = np.clip(v, -APPROACH_SPEED, APPROACH_SPEED)
                omega = np.clip(omega, -MAX_ANGULAR_SPEED, MAX_ANGULAR_SPEED)
                robot.set_mobile_base_speed_and_gripper_power(v, omega, 0)

        elif mission_phase == "MANIPULATE":
            print("Performing manipulation action...")
            if not is_navigating_to_dropoff:
                perform_pickup(robot)
            else:
                perform_dropoff(robot)

            print("Backing up...")
            start_time = time.time()
            while time.time() - start_time < BACKUP_TIME:
                robot.set_mobile_base_speed_and_gripper_power(BACKUP_SPEED, 0, 0)
                time.sleep(0.05)
                
            start_time = time.time()
            while time.time() - start_time < 0.2:
                robot.set_mobile_base_speed_and_gripper_power(0, 0, 0)
                time.sleep(0.05)

            if not is_navigating_to_dropoff:
                is_navigating_to_dropoff = True
                mission_phase = "NAVIGATE"
                print("Pickup complete. Proceeding to dropoff location...")
            else:
                print("Dropoff complete. Mission successful!")
                robot.set_leds(*COLOR_GREEN)
                time.sleep(3)
                break