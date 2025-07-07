from robot import Robot
import time


class MobileManipulatorUnicycle(Robot):
    def __init__(self, robot_id, backend_server_ip=None):
        super(MobileManipulatorUnicycle, self).__init__(robot_id, backend_server_ip)

    def set_arm_pose(self, x, y, wait_s=2.6): # Not recommended to use a smaller wait time than this
        super(MobileManipulatorUnicycle, self).step([0., 0., 0., x, y, 1., 0.])
        if wait_s > 0.:
            time.sleep(wait_s)

    def set_mobile_base_speed_and_gripper_power(self, v : float, omega : float, gripper_power : float):
        super(MobileManipulatorUnicycle, self).step([v, 0, omega, 0., 0., 0., gripper_power])

