import numpy as np
import cvxpy as cp
import time
import math

class project:
    def __init__(self, robot):
        self.robot = robot
        self.pose = None
        self.pick_loc = None
        self.drop_loc = None 
        self.obs = None
        self.safety_radius = 1  # Safety radius for CBF
        self.alpha = 1.0  # CLF gain
        self.gamma = 1.0  # CBF gain
        self.robot_radius = 0.5  # Robot radius for obstacle avoidance
        self.picked_up = False
        self.finished = False
        self.counter = 0
        # self.trans_obs_from_pick_up_drop_off = []

    def initialize(self):
        self.pose,self.pick_loc,self.drop_loc,self.obs = self.robot.get_poses()
        while not self.pose or not self.pick_loc or not self.drop_loc or not self.obs:
            self.pose,self.pick_loc,self.drop_loc,self.obs = self.robot.get_poses()
        # Normalize the robot's orientation to be within [0, 2*pi)
        if self.pose[2] >= np.pi * 2:
            self.pose[2] -= np.pi * 2
        if self.pose[2] < 0:
            self.pose[2] += np.pi * 2
        # self.trans_obs_from_pick_up_drop_off.append(self.pick_loc)
        # self.trans_obs_from_pick_up_drop_off.append(self.drop_loc)
        # print(self.trans_obs_from_pick_up_drop_off)
        

    def update(self):
        self.pose, self.pick_loc, self.drop_loc, self.obs = self.robot.get_poses()
        # Normalize the robot's orientation to be within [0, 2*pi)
        if self.pose[2] >= np.pi * 2:
            self.pose[2] -= np.pi * 2
        if self.pose[2] < 0:
            self.pose[2] += np.pi * 2
        # self.trans_obs_from_pick_up_drop_off.append(self.pick_loc)
        # self.trans_obs_from_pick_up_drop_off.append(self.drop_loc)
    @staticmethod
    def clf_cbf_qp_controller(state, goal, obstacles, alpha, gamma):
        x, y, theta = state
        x_d, y_d = goal

        # Compute position errors
        ex = x - x_d
        ey = y - y_d
        V = 0.5 * (ex**2 + ey**2)
        slack = cp.Variable(nonneg=True)

        # Desired heading to face the goal
        theta_d = np.arctan2(y_d - y, x_d - x)
        e_theta = theta - theta_d
        e_theta = np.arctan2(np.sin(e_theta), np.cos(e_theta))  # wrap to [-pi, pi]

        # Heading error CLF
        V_theta = 0.5 * e_theta**2

        # Define optimization variables
        v = cp.Variable()
        omega = cp.Variable()

        # Compute derivatives of CLFs
        dotV = ex * v * np.cos(theta) + ey * v * np.sin(theta)
        dotV_theta = e_theta * omega

        # Combined CLF constraint
        clf_constraint = dotV + dotV_theta + alpha * (V + V_theta) <= slack

        # CBF constraints for each obstacle
        cbf_constraints = []
        omega_min = -math.radians(30)
        omega_max = math.radians(30)

        cbf_constraints += [
            omega >= omega_min,
            omega <= omega_max
        ]
        cbf_constraints += [
            v >= 0,
            v <= .1
        ]
        
        
        
        omega_bias = 0.0
        k_repulse = 20  # strength of repulsion
        epsilon = 1e-2   # to avoid divide-by-zero
        margin = 0.05
        ROBOT_SIZE = [0.24, 0.32]
        safety_radius = (ROBOT_SIZE[0] + margin)**2 + (ROBOT_SIZE[1] + margin)**2

        for i in range(len(obstacles)):
            ox, oy = obstacles[i][0], obstacles[i][1]
            dx = x - ox
            dy = y - oy 
            h = dx**2 + dy**2 - safety_radius**2
            doth = 2 * dx * v * np.cos(theta) + 2 * dy * v * np.sin(theta)
            cbf_constraints.append(doth + gamma * h >= 0)

        
        for ox, oy,_ in obstacles:
            dx = x - ox 
            dy = y - oy 
            dist = np.sqrt(dx**2 + dy**2)
            if dist < safety_radius + 0.25:  # only trigger if close to danger
                angle_to_obs = np.arctan2(dy, dx)
                angle_diff = np.arctan2(np.sin(theta - angle_to_obs), np.cos(theta - angle_to_obs))
                repulsion_strength = k_repulse * angle_diff / (dist - safety_radius + epsilon)
                omega_bias += repulsion_strength
        # for ox,oy in trans_obs_from_pick_up_drop_off:
        #     dx = x - ox
        #     dy = y - oy
        #     h = dx**2 + dy**2 - 0.4**2
        #     doth = 2 * dx * v * np.cos(theta) + 2 * dy * v * np.sin(theta)
        #     cbf_constraints.append(doth + gamma * h >=0)

        # cbf_constraints.append(v**2 <= 4)
        # cbf_constraints.append(omega**2 <= 1)  # Ensure non-negative linear velocity

        # Objective: minimize control effort
        # objective = cp.Minimize(v**2 + omega**2 + 1000 * slack)
        objective = cp.Minimize(
            v**2 + (omega + omega_bias)**2 + 1000 * slack
            )

        # Solve the QP
        constraints = [clf_constraint] + cbf_constraints

        prob = cp.Problem(objective, constraints)
        prob.solve(solver=cp.SCS)

        if prob.status != cp.OPTIMAL:
            print("QP failed, zero control used")
            return np.array([0.0, 0.0])

        return np.array([v.value, omega.value])
    
    def move_robot(self, v, omega,duration,forward=1):
        # Convert linear and angular velocities to robot's speed and turn rate
        speed = v * forward  
        turn_rate = omega
        
        # Update the robot's pose using the unicycle model
        start=time.time()
        while time.time() - start < duration: 
             self.robot.set_mobile_base_speed_and_gripper_power(v=speed, omega=turn_rate, gripper_power=0.0)

    def pickup_object(self):
        print("Picking up object...")
        self.move_robot(1, 0,.5, forward=-1)  # Move towards the object
    
    def dropoff_object(self):
        print("Dropping off object...")
        self.move_robot(1, 0,.5, forward=-1)

    def final_adjust(self, t_loc):
        print(f"DEBUG: Robot pose (world): ({self.pose[0]:.2f}, {self.pose[1]:.2f}, {np.degrees(self.pose[2]):.1f}°)")
        print(f"DEBUG: Target for rotation (world): ({t_loc[0]:.2f}, {t_loc[1]:.2f})")

        while True:
            self.update()
            if self.pose is None: # Safely handle missing pose
                print("DEBUG: Pose is None during rotation, waiting...")
                time.sleep(self.control_loop_delay)
                continue

            dx = t_loc[0] - self.pose[0]
            dy = t_loc[1] - self.pose[1]
            desired_theta = math.atan2(dy, dx)
            
            delta_theta = desired_theta - self.pose[2]
            delta_theta = (delta_theta + math.pi) % (2 * math.pi) - math.pi

            # === START ROTATION FIXES: MINIMUM SPEED THRESHOLD AND UPDATED TOLERANCE ===
            if abs(delta_theta) < 0.015:
                print(f"DEBUG: Rotation complete. Final delta_theta: {np.degrees(delta_theta):.1f}°")
                break

            omega = 2.5 * delta_theta

            # Apply minimum speed threshold for small errors
            if abs(omega) < 0.005:
                omega = math.copysign(0.005, omega)

            omega = max(-5, min(5, omega))
            # === END ROTATION FIXES ===

            self.robot.set_mobile_base_speed_and_gripper_power(v=0.0, omega=omega, gripper_power=0.0)
            time.sleep(.01) # Use consistent delay

    
    def run(self):
        self.initialize()
        while not self.finished:
            self.update()
            if not self.picked_up:
                goal = np.array(self.pick_loc[:2])
            else:
                goal = np.array(self.drop_loc[:2])
            
            # Check if the robot is close enough to pick up the object
            if not self.picked_up and np.linalg.norm(np.array(self.pose[:2]) - goal) < 0.5:
                self.final_adjust(goal)
                self.pickup_object()
                self.picked_up = True
                print("Object picked up.")
            
            # Check if the robot is close enough to drop off the object
            elif self.picked_up and np.linalg.norm(np.array(self.pose[:2]) - goal) < 0.5:
                self.final_adjust(goal)
                self.dropoff_object()
                self.finished = True
                print("Object dropped off.")
            
            # Get control inputs using CLF-CBF QP controller
            control = self.clf_cbf_qp_controller(self.pose, goal, self.obs, alpha=self.alpha, gamma=self.gamma)
            v, omega = control
            print(f"Control: v={v:.2f}, omega={omega:.2f}")
            

            self.move_robot(v, omega,.05)
        