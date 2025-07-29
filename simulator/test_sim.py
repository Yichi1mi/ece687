from mobile_manipulator_unicycle_sim import MobileManipulatorUnicycleSim
from avoid_obstacle_sim import avoid_obstacles
import time

# Two options to initialize the simulator
# (i) random robot and object locations
robot = MobileManipulatorUnicycleSim(robot_id=1)
# (ii) specified robot and object locations
# robot = MobileManipulatorUnicycleSim(robot_id=1, robot_pose=[0.0, 0.0, 0.0], pickup_location=[0.75, 0.75], dropoff_location=[-0.75, -0.75], obstacles_location=[[0.5, 0.5], [-0.5, -0.5]])

avoid_obstacles(robot)

