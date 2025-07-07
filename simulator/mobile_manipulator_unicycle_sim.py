import math
import random
import time
import matplotlib.patches as patches
import matplotlib.pyplot as plt
import numpy as np

def generate_non_overlapping_positions(n, radius, env_size):
    positions = []
    max_attempts = 1000
    attempts = 0
    margin = radius + 0.05
    while len(positions) < n and attempts < max_attempts:
        x = random.uniform(-env_size / 2.0 + margin, env_size / 2.0 - margin)
        y = random.uniform(-env_size / 2.0 + margin, env_size / 2.0 - margin)
        valid = True
        for px, py in positions:
            if math.hypot(x - px, y - py) < 2 * radius:
                valid = False
                break
        if valid:
            positions.append([x, y])
        attempts += 1
    if len(positions) < n:
        raise Exception("Could not generate non-overlapping obstacle positions.")
    return positions

def generate_safe_point(env_size, margin):
    limit = env_size / 2.0 - margin
    x = random.uniform(-limit, limit)
    y = random.uniform(-limit, limit)
    return [x, y]

class MobileManipulatorUnicycleSim:
    def __init__(self, robot_id, backend_server_ip=None, robot_pose=[], pickup_location=[], dropoff_location=[], obstacles_location=[]):
        self.MAX_LINEAR_SPEED = 0.1
        self.MAX_ANGULAR_SPEED = math.radians(30)
        self.TIMEOUT_SET_MOBILE_BASE_SPEED = 0
        self.TIMEOUT_GET_POSES = 0
        self.ENV_SIZE = 5
        self.ROBOT_SIZE = [0.24, 0.32]
        self.GRIPPER_SIZE = 0.1
        self.LOC_RADIUS = 0.1

        if robot_pose: self.robot_pose = robot_pose
        else:
            robot_x, robot_y = generate_safe_point(self.ENV_SIZE, 0.20)
            angle_to_center = math.atan2(-robot_y, -robot_x)
            angle_offset = random.uniform(-math.pi / 6, math.pi / 6)
            robot_theta = angle_to_center + angle_offset
            self.robot_pose = [robot_x, robot_y, robot_theta]

        if pickup_location: self.pickup_location = pickup_location
        else: self.pickup_location = generate_safe_point(self.ENV_SIZE, 0.40)

        if dropoff_location: self.dropoff_location = dropoff_location
        else: self.dropoff_location = generate_safe_point(self.ENV_SIZE, 0.40)

        if obstacles_location: self.obstacles_location = obstacles_location
        else:
            radius = max(self.ROBOT_SIZE) * 0.6
            self.obstacles_location = generate_non_overlapping_positions(9, radius, self.ENV_SIZE)

        self.obstacle_poses = [[self.obstacles_location[i][0], self.obstacles_location[i][1], 2 * math.pi * random.random()] for i in range(len(self.obstacles_location))]
        self.obstacle_speeds = [[0.07 + 0.03 * random.random(), (2 * random.random() - 1) * 30] for _ in range(len(self.obstacles_location))]
        self.obstacle_states = ['moving' if random.random() > 0.5 else 'stopped' for _ in range(len(self.obstacles_location))]
        self.obstacle_state_timers = [random.uniform(2.0, 5.0) for _ in range(len(self.obstacles_location))]
        self.last_time_set_mobile_base_speed = int(round(time.time()*1000))
        self.last_time_get_poses = int(round(time.time()*1000))
        self.figure, self.axes, self.patches = [], [], []
        self.__init_plot()
    
    def __init_plot(self):
        self.figure, self.axes = plt.subplots()
        p_env = patches.Rectangle(np.array([-self.ENV_SIZE / 2.0, -self.ENV_SIZE / 2.0]), self.ENV_SIZE, self.ENV_SIZE, fill=False)
        p_pu_loc = patches.Circle(np.array(self.pickup_location), radius=self.LOC_RADIUS, facecolor='b')
        p_do_loc = patches.Circle(np.array(self.dropoff_location), radius=self.LOC_RADIUS, facecolor='g')
        R = np.array([[0.0, 1.0], [-1.0, 0.0]]) @ np.array([[math.cos(self.robot_pose[2]), -math.sin(self.robot_pose[2])], [math.sin(self.robot_pose[2]), math.cos(self.robot_pose[2])]])
        t = np.array([self.robot_pose[0], self.robot_pose[1]])
        p_robot = patches.Polygon(t + np.array([0, self.ROBOT_SIZE[1]]) @ R.T / 2.0 + (np.array([[-self.ROBOT_SIZE[0] / 2.0, 0.0], [self.ROBOT_SIZE[0] / 2.0, 0.0], [self.ROBOT_SIZE[0] / 2.0, self.ROBOT_SIZE[1]], [-self.ROBOT_SIZE[0] / 2.0, self.ROBOT_SIZE[1]]]) - np.array([0, self.ROBOT_SIZE[1]]) / 2.0) @ R.T, facecolor='k')
        p_gripper = patches.Polygon(t + np.array([0, self.ROBOT_SIZE[1]]) @ R.T / 2.0 + (np.array([0, self.ROBOT_SIZE[1]]) + np.array([[-self.GRIPPER_SIZE / 2.0, 0.0], [self.GRIPPER_SIZE / 2.0, 0.0], [self.GRIPPER_SIZE / 2.0, self.GRIPPER_SIZE], [0.8 * self.GRIPPER_SIZE / 2.0, self.GRIPPER_SIZE], [0.8 * self.GRIPPER_SIZE / 2.0, 0.0], [-0.8 * self.GRIPPER_SIZE / 2.0, 0.0], [-0.8 * self.GRIPPER_SIZE / 2.0, self.GRIPPER_SIZE], [-self.GRIPPER_SIZE / 2.0, self.GRIPPER_SIZE], [-self.GRIPPER_SIZE / 2.0, 0.0]]) - np.array([0, self.ROBOT_SIZE[1]]) / 2.0) @ R.T, facecolor='k')
        self.patches.extend([p_pu_loc, p_do_loc, p_robot, p_gripper])
        self.axes.add_patch(p_env)
        for p in self.patches: self.axes.add_patch(p)
        for obs_pos in self.obstacles_location:
            obstacle_circle = patches.Circle(np.array(obs_pos), radius=max(self.ROBOT_SIZE) * 0.5, facecolor='r')
            self.patches.append(obstacle_circle)
            self.axes.add_patch(obstacle_circle)
        self.axes.set_xlim(-0.6*self.ENV_SIZE, 0.6*self.ENV_SIZE)
        self.axes.set_ylim(-0.6*self.ENV_SIZE, 0.6*self.ENV_SIZE)
        self.axes.set_axis_off()
        self.axes.axis('equal')
        plt.ion()
        plt.show()

    def __update_plot(self):
        R = np.array([[0.0, 1.0], [-1.0, 0.0]]) @ np.array([[math.cos(self.robot_pose[2]), -math.sin(self.robot_pose[2])], [math.sin(self.robot_pose[2]), math.cos(self.robot_pose[2])]])
        t = np.array([self.robot_pose[0], self.robot_pose[1]])
        xy_robot = t + np.array([0, self.ROBOT_SIZE[1]]) @ R.T / 2.0 + (np.array([[-self.ROBOT_SIZE[0] / 2.0, 0.0], [self.ROBOT_SIZE[0] / 2.0, 0.0], [self.ROBOT_SIZE[0] / 2.0, self.ROBOT_SIZE[1]], [-self.ROBOT_SIZE[0] / 2.0, self.ROBOT_SIZE[1]]]) - np.array([0, self.ROBOT_SIZE[1]]) / 2.0) @ R.T
        xy_gripper = t + np.array([0, self.ROBOT_SIZE[1]]) @ R.T / 2.0 + (np.array([0, self.ROBOT_SIZE[1]]) + np.array([[-self.GRIPPER_SIZE / 2.0, 0.0], [self.GRIPPER_SIZE / 2.0, 0.0], [self.GRIPPER_SIZE / 2.0, self.GRIPPER_SIZE], [0.8 * self.GRIPPER_SIZE / 2.0, self.GRIPPER_SIZE], [0.8 * self.GRIPPER_SIZE / 2.0, 0.0], [-0.8 * self.GRIPPER_SIZE / 2.0, 0.0], [-0.8 * self.GRIPPER_SIZE / 2.0, self.GRIPPER_SIZE], [-self.GRIPPER_SIZE / 2.0, self.GRIPPER_SIZE], [-self.GRIPPER_SIZE / 2.0, 0.0]]) - np.array([0, self.ROBOT_SIZE[1]]) / 2.0) @ R.T
        self.patches[0].center = self.pickup_location
        self.patches[1].center = self.dropoff_location
        self.patches[2].xy = xy_robot
        self.patches[3].xy = xy_gripper
        
        obstacle_radius_for_collision = max(self.ROBOT_SIZE) * 0.6
        dt = 0.05

        for i in range(len(self.obstacle_poses)):
            self.obstacle_state_timers[i] -= dt
            if self.obstacle_state_timers[i] <= 0:
                self.obstacle_states[i] = 'stopped' if self.obstacle_states[i] == 'moving' else 'moving'
                self.obstacle_state_timers[i] = random.uniform(2.0, 5.0)

            if self.obstacle_states[i] == 'moving':
                x, y, theta = self.obstacle_poses[i]
                v, omega_deg = self.obstacle_speeds[i]
                new_x, new_y = x + v * math.cos(theta) * dt, y + v * math.sin(theta) * dt
                bounced = False

                # --- 新增：避让目标点 ---
                TARGET_SAFE_RADIUS = self.LOC_RADIUS + obstacle_radius_for_collision + 0.05
                if math.hypot(new_x - self.pickup_location[0], new_y - self.pickup_location[1]) < TARGET_SAFE_RADIUS or \
                   math.hypot(new_x - self.dropoff_location[0], new_y - self.dropoff_location[1]) < TARGET_SAFE_RADIUS:
                    theta += math.pi
                    bounced = True
                
                # 撞墙反弹
                if not bounced and (abs(new_x) + obstacle_radius_for_collision > self.ENV_SIZE / 2.0 or abs(new_y) + obstacle_radius_for_collision > self.ENV_SIZE / 2.0):
                    theta += math.pi
                    bounced = True

                # 障碍物之间碰撞反弹
                if not bounced:
                    for j in range(len(self.obstacle_poses)):
                        if i == j: continue
                        ox, oy, _ = self.obstacle_poses[j]
                        if math.hypot(new_x - ox, new_y - oy) < 2 * obstacle_radius_for_collision:
                            theta += (random.random() - 0.5) * math.pi
                            bounced = True
                            break
                
                if bounced: new_x, new_y = x, y
                theta += math.radians(omega_deg) * dt
                self.obstacle_poses[i] = [new_x, new_y, theta]
                self.obstacles_location[i] = [new_x, new_y]

            self.patches[4 + i].center = (self.obstacles_location[i][0], self.obstacles_location[i][1])

        self.figure.canvas.draw_idle()
        self.figure.canvas.flush_events()

    def set_arm_pose(self, x, y, wait_s=2.6): pass

    def set_mobile_base_speed_and_gripper_power(self, v, omega, gripper_power):
        delta_time = int(round(time.time()*1000)) - self.last_time_set_mobile_base_speed
        if delta_time > self.TIMEOUT_SET_MOBILE_BASE_SPEED:
            v, omega = self.__saturate_speeds(v, omega)
            self.robot_pose[0] += (v * math.cos(self.robot_pose[2])) * delta_time / 1000.0
            self.robot_pose[1] += (v * math.sin(self.robot_pose[2])) * delta_time / 1000.0
            self.robot_pose[2] += omega * delta_time / 1000.0
            self.last_time_set_mobile_base_speed = int(round(time.time()*1000))
            self.__update_plot()

    def set_leds(self, red_level, green_level, blue_level): pass

    def __saturate_speeds(self, v, omega):
        v = max(-self.MAX_LINEAR_SPEED, min(v, self.MAX_LINEAR_SPEED))
        omega = max(-self.MAX_ANGULAR_SPEED, min(omega, self.MAX_ANGULAR_SPEED))
        return v, omega
    
    def get_poses(self):
        while int(round(time.time()*1000)) - self.last_time_get_poses < self.TIMEOUT_GET_POSES: pass
        self.last_time_get_poses = int(round(time.time()*1000))
        return [self.robot_pose, self.pickup_location, self.dropoff_location, self.obstacle_poses]