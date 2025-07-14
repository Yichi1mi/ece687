import math
import time
from mobile_manipulator_unicycle import MobileManipulatorUnicycleSim
from clf_cbf_qp_controller import clf_cbf_qp_controller

# === LED COLOR DEFINITIONS (R, G, B) ===
COLOR_RED = (255, 0, 0)
COLOR_YELLOW = (255, 255, 0)
COLOR_BLUE = (0, 0, 255)
COLOR_GREEN = (0, 255, 0)

def perform_pickup(robot: MobileManipulatorUnicycleSim):
    """
    Executes the full, detailed sequence for picking up an object.
    """
    print("Initiating pickup sequence...")
    robot.set_leds(*COLOR_YELLOW)  # Set LED to Yellow for manipulation.

    # Step 1: Move the arm to a safe pre-grasp position.
    print("  [Step 1/5] Moving arm to pre-grasp position (100, 50)")
    robot.set_arm_pose(0, 0)
    time.sleep(1.5)

    # Step 2: Open the gripper to prepare for grasping.
    print("  [Step 2/5] Opening gripper")
    robot.set_mobile_base_speed_and_gripper_power(v=0.0, omega=0.0, gripper_power=1.0)
    time.sleep(2.0)

    # Step 3: Lower the arm to the grasp position.
    print("  [Step 3/5] Lowering arm to grasp position (200, -60)")
    robot.set_arm_pose(200, -60)
    time.sleep(1.5)

    # Step 4: Close the gripper to secure the object.
    print("  [Step 4/5] Closing gripper")
    start_time = time.time()
    while time.time() - start_time < 2.5:
        robot.set_mobile_base_speed_and_gripper_power(v=0.0, omega=0.0, gripper_power=-1.0)
        time.sleep(0.05)

    # Step 5: Lift the arm, carrying the object.
    print("  [Step 5/5] Lifting object")
    robot.set_arm_pose(100, 50)
    time.sleep(1.5)

    print("Pickup sequence complete.")


def perform_dropoff(robot: MobileManipulatorUnicycleSim):
    """
    Executes the sequence for dropping off an object.
    """
    print("Initiating dropoff sequence...")
    robot.set_leds(*COLOR_YELLOW)  # Set LED to Yellow for manipulation.

    # Step 1: Move the arm to the dropoff position.
    print("  [Step 1/3] Lowering arm to dropoff position (200, -60)")
    robot.set_arm_pose(200, -60)
    time.sleep(1.5)

    # Step 2: Open the gripper to release the object.
    print("  [Step 2/3] Opening gripper to release object")
    robot.set_mobile_base_speed_and_gripper_power(v=0.0, omega=0.0, gripper_power=1.0)
    time.sleep(2.0)

    # Step 3: Retract the arm to a safe/home position.
    print("  [Step 3/3] Retracting arm")
    robot.set_arm_pose(0, 0)
    time.sleep(1.5)

    print("Dropoff sequence complete.")


def run_pick_and_place_mission(robot: MobileManipulatorUnicycleSim):
    """
    Main mission control function.
    """
    is_navigating_to_dropoff = False
    REACHED_THRESHOLD = 0.02
    BACKUP_TIME = 2.0
    BACKUP_SPEED = -0.08

    while True:
        robot_pose, pickup_loc, dropoff_loc, _ = robot.get_poses()

        if not all([robot_pose, pickup_loc, dropoff_loc]):
            print("Warning: Pose data lost. Pausing execution and waiting for data...")
            robot.set_leds(*COLOR_RED) # Red for data loss error
            time.sleep(0.5)
            continue

        goal = dropoff_loc if is_navigating_to_dropoff else pickup_loc

        gripper_x = robot_pose[0] + 0.4 * math.cos(robot_pose[2])
        gripper_y = robot_pose[1] + 0.4 * math.sin(robot_pose[2])
        distance_to_goal = math.hypot(gripper_x - goal[0], gripper_y - goal[1])

        if distance_to_goal < REACHED_THRESHOLD:
            print(f"Goal reached at {goal}")
            robot.set_mobile_base_speed_and_gripper_power(0, 0, 0)

            if not is_navigating_to_dropoff:
                perform_pickup(robot)
            else:
                perform_dropoff(robot)

            print("Backing up to create maneuvering space...")
            start_time = time.time()
            while time.time() - start_time < BACKUP_TIME:
                robot.set_mobile_base_speed_and_gripper_power(BACKUP_SPEED, 0, 0)
                time.sleep(0.05)
            robot.set_mobile_base_speed_and_gripper_power(0, 0, 0)

            if not is_navigating_to_dropoff:
                is_navigating_to_dropoff = True
                print("Pickup complete. Proceeding to dropoff location...")
            else:
                print("Dropoff complete. Mission successful!")
                robot.set_leds(*COLOR_GREEN) # Green for mission complete
                time.sleep(5) # Keep green light on for 5 seconds
                break

            continue

        robot.set_leds(*COLOR_BLUE)  # Set LED to Blue to indicate navigation
        clf_cbf_qp_controller(robot, goal)