from mobile_manipulator_unicycle import MobileManipulatorUnicycle


robot = MobileManipulatorUnicycle(robot_id=1, backend_server_ip="192.168.0.2")

while not task_done:
    # get poses
    poses = robot.get_poses()
    
    # compute control inputs (v, omega, gripper power, arm pose, leds)
    # ...
    
    # send control inputs to the robot
    # robot.set_mobile_base_speed_and_gripper_power(v=0.5, omega=0.0, gripper_power=1.0)
    # robot.set_arm_pose(100, 0)
    # robot.set_leds(255, 128, 64)