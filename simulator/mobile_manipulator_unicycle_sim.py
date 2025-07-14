import math
import random
import time
import matplotlib.patches as patches
import matplotlib.pyplot as plt
import numpy as np

# === 修改点 1/3 开始: 重写生成逻辑以满足更复杂的约束 ===
def generate_non_overlapping_poses(n, env_size, robot_size, target_points, obs_safe_dist, target_safe_dist):
    """
    生成一组不重叠且满足所有安全距离约束的机器人位姿。
    - n: 要生成的位姿数量
    - env_size: 环境大小
    - robot_size: 机器人尺寸，用于计算d_center
    - target_points: 需要保持安全距离的目标点列表
    - obs_safe_dist: 障碍物之间的最小几何中心距离
    - target_safe_dist: 障碍物几何中心与目标点之间的最小距离
    """
    poses = []
    max_attempts_per_pose = 1000
    d_center = robot_size[1] / 2.0

    for i in range(n):
        attempts = 0
        while attempts < max_attempts_per_pose:
            # 随机生成一个完整的位姿 (后轴坐标和朝向)
            x_r = random.uniform(-env_size / 2.0, env_size / 2.0)
            y_r = random.uniform(-env_size / 2.0, env_size / 2.0)
            theta = random.uniform(-math.pi, math.pi)
            
            # 立刻计算其几何中心
            x_c = x_r + d_center * math.cos(theta)
            y_c = y_r + d_center * math.sin(theta)

            valid = True
            # 1. 检查与环境边界 (使用包围圆近似)
            bounding_radius = math.hypot(robot_size[0], robot_size[1]) / 2.0
            if not (-env_size/2 + bounding_radius < x_c < env_size/2 - bounding_radius and
                    -env_size/2 + bounding_radius < y_c < env_size/2 - bounding_radius):
                valid = False

            # 2. 检查与已生成障碍物的距离 (obs_safe_dist)
            if valid:
                for other_pose in poses:
                    other_xc, other_yc = other_pose[1] # 获取已存储的几何中心
                    if math.hypot(x_c - other_xc, y_c - other_yc) < obs_safe_dist:
                        valid = False
                        break
            # 3. 检查与目标点的距离 (target_safe_dist)
            if valid:
                for target_p in target_points:
                    if math.hypot(x_c - target_p[0], y_c - target_p[1]) < target_safe_dist:
                        valid = False
                        break
            if valid:
                # 存储后轴位姿和几何中心位姿
                poses.append(([x_r, y_r, theta], (x_c, y_c)))
                break
            attempts += 1
        if attempts == max_attempts_per_pose:
            raise Exception(f"无法为第 {i+1} 个障碍物生成满足所有约束的位置。")

    # 返回后轴位姿列表
    return [p[0] for p in poses]
# === 修改点 1/3 结束 ===


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
        self.ROBOT_SIZE = [0.24, 0.32] # width, length
        self.GRIPPER_SIZE = 0.1
        self.LOC_RADIUS = 0.1
        self.d_center = self.ROBOT_SIZE[1] / 2.0


        # 1. 定义角落坐标和中心取货点
        self.pickup_location = [0.0, 0.0]
        corner_dist_from_center = self.ENV_SIZE / 2.0 - 0.5
        corners = [
            [corner_dist_from_center, corner_dist_from_center],      # Top-right
            [-corner_dist_from_center, corner_dist_from_center],     # Top-left
            [-corner_dist_from_center, -corner_dist_from_center],    # Bottom-left
            [corner_dist_from_center, -corner_dist_from_center]      # Bottom-right
        ]

        # 2. 在随机角落生成机器人位姿
        random.shuffle(corners) # 随机打乱角落顺序
        robot_center_loc = corners.pop(0) # 为机器人取出一个角落
        theta = random.uniform(-math.pi, math.pi)

        # 根据几何中心位置反向计算后轴位置
        x_c, y_c = robot_center_loc
        x_r = x_c - self.d_center * math.cos(theta)
        y_r = y_c - self.d_center * math.sin(theta)
        self.robot_pose = [x_r, y_r, theta]

        # 3. 从剩下的三个角落中随机选择一个作为放置点
        self.dropoff_location = random.choice(corners)

        # 4. 根据新约束生成障碍物
        # 障碍物需要与机器人中心、取货点、放置点保持至少0.4m的距离
        points_to_avoid = [robot_center_loc, self.pickup_location, self.dropoff_location]
        
        if obstacles_location:
            # 如果提供了外部位置，则直接使用（这将覆盖上述规则）
            self.obstacle_poses = [[loc[0], loc[1], random.uniform(-math.pi, math.pi)] for loc in obstacles_location]
        else:
            # 使用生成函数并传入新的约束条件
            self.obstacle_poses = generate_non_overlapping_poses(
                n=7,
                env_size=self.ENV_SIZE,
                robot_size=self.ROBOT_SIZE,
                target_points=points_to_avoid,
                obs_safe_dist=0.8,  # 障碍物之间的最小中心距离
                target_safe_dist=0.6  # 与机器人/目标点的最小中心距离
            )
        
        # === 生成逻辑修改结束 ===

        # self.obstacle_speeds = [[0, 0] for _ in self.obstacle_poses]
        self.obstacle_speeds = [[0.07 + 0.03 * random.random(), (2 * random.random() - 1) * math.radians(30)] for _ in self.obstacle_poses]
        
        self.obstacle_states = ['moving' if random.random() > 0.5 else 'stopped' for _ in self.obstacle_poses]
        self.obstacle_state_timers = [random.uniform(2.0, 5.0) for _ in self.obstacle_poses]
        self.obstacle_turn_target_angles = [None] * len(self.obstacle_poses)
        self.obstacle_turn_centers = [None] * len(self.obstacle_poses)
        
        self.last_time_set_mobile_base_speed = int(round(time.time()*1000))
        self.last_time_get_poses = int(round(time.time()*1000))
        self.figure, self.axes, self.patches = [], [], []
        self.__init_plot()



    def __calculate_model_vertices(self, pose, size, gripper_size):
        rear_axle_x, rear_axle_y, theta = pose

        # Step 1: 计算几何中心作为 anchor
        x_c = rear_axle_x + self.d_center * math.cos(theta)
        y_c = rear_axle_y + self.d_center * math.sin(theta)
        t = np.array([x_c, y_c])  # 绘图 anchor 改为几何中心

        # Step 2: 构造旋转矩阵（不变）
        R = np.array([[0.0, 1.0], [-1.0, 0.0]]) @ np.array([
            [math.cos(theta), -math.sin(theta)],
            [math.sin(theta),  math.cos(theta)]
        ])

        # Step 3: 绘图顶点统一绕“中心”展开
        body_shape = np.array([
            [-size[0] / 2.0, -size[1] / 2.0],
            [ size[0] / 2.0, -size[1] / 2.0],
            [ size[0] / 2.0,  size[1] / 2.0],
            [-size[0] / 2.0,  size[1] / 2.0]
        ])
        body_verts = t + (body_shape @ R.T)

        # Step 4: gripper 还是放在前方，但也以几何中心为基准绘图
        gripper_offset = np.array([0.0, size[1] / 2.0])
        gripper_shape = np.array([
            [-gripper_size / 2.0, 0.0],
            [ gripper_size / 2.0, 0.0],
            [ gripper_size / 2.0, gripper_size],
            [ 0.8 * gripper_size / 2.0, gripper_size],
            [ 0.8 * gripper_size / 2.0, 0.0],
            [-0.8 * gripper_size / 2.0, 0.0],
            [-0.8 * gripper_size / 2.0, gripper_size],
            [-gripper_size / 2.0, gripper_size],
            [-gripper_size / 2.0, 0.0]
        ])
        gripper_verts = t + ((gripper_shape + gripper_offset) @ R.T)

        return body_verts, gripper_verts

    def __init_plot(self):
        self.figure, self.axes = plt.subplots()
        p_env = patches.Rectangle(np.array([-self.ENV_SIZE / 2.0, -self.ENV_SIZE / 2.0]), self.ENV_SIZE, self.ENV_SIZE, fill=False)
        self.axes.add_patch(p_env)
        
        p_pu_loc = patches.Circle(np.array(self.pickup_location), radius=self.LOC_RADIUS, facecolor='b')
        p_do_loc = patches.Circle(np.array(self.dropoff_location), radius=self.LOC_RADIUS, facecolor='g')
        self.patches.extend([p_pu_loc, p_do_loc])
        self.axes.add_patch(p_pu_loc)
        self.axes.add_patch(p_do_loc)

        xy_robot, xy_gripper = self.__calculate_model_vertices(self.robot_pose, self.ROBOT_SIZE, self.GRIPPER_SIZE)

        p_robot_body = patches.Polygon(xy_robot, facecolor='k')
        p_robot_gripper = patches.Polygon(xy_gripper, facecolor='k')
        self.patches.extend([p_robot_body, p_robot_gripper])
        self.axes.add_patch(p_robot_body)
        self.axes.add_patch(p_robot_gripper)
        
        for obs_pose in self.obstacle_poses:
            xy_obs_body, xy_obs_gripper = self.__calculate_model_vertices(obs_pose, self.ROBOT_SIZE, self.GRIPPER_SIZE)
            p_obs_body = patches.Polygon(xy_obs_body, facecolor='r')
            p_obs_gripper = patches.Polygon(xy_obs_gripper, facecolor='r')
            self.patches.extend([p_obs_body, p_obs_gripper])
            self.axes.add_patch(p_obs_body)
            self.axes.add_patch(p_obs_gripper)

        self.axes.set_xlim(-0.6*self.ENV_SIZE, 0.6*self.ENV_SIZE)
        self.axes.set_ylim(-0.6*self.ENV_SIZE, 0.6*self.ENV_SIZE)
        self.axes.set_axis_off()
        self.axes.axis('equal')
        plt.ion()
        plt.show()

    def __update_plot(self):
        dt = 0.05
        self.patches[0].center = self.pickup_location
        self.patches[1].center = self.dropoff_location
        
        xy_robot, xy_gripper = self.__calculate_model_vertices(self.robot_pose, self.ROBOT_SIZE, self.GRIPPER_SIZE)

        self.patches[2].xy = xy_robot
        self.patches[3].xy = xy_gripper
        
        obs_bounding_radius = math.hypot(self.ROBOT_SIZE[0], self.ROBOT_SIZE[1]) / 2.0

        for i in range(len(self.obstacle_poses)):
            if self.obstacle_states[i] != 'turning':
                self.obstacle_state_timers[i] -= dt
                if self.obstacle_state_timers[i] <= 0:
                    self.obstacle_states[i] = 'stopped' if self.obstacle_states[i] == 'moving' else 'moving'
                    self.obstacle_state_timers[i] = random.uniform(2.0, 5.0)

            current_state = self.obstacle_states[i]
            
            if current_state == 'turning':
                if self.obstacle_turn_centers[i] is None:
                    x_r, y_r, theta = self.obstacle_poses[i]
                    x_c = x_r + self.d_center * math.cos(theta)
                    y_c = y_r + self.d_center * math.sin(theta)
                    
                    self.obstacle_turn_centers[i] = (x_c, y_c)

                x_c, y_c = self.obstacle_turn_centers[i]
                _, _, theta = self.obstacle_poses[i]
                target_angle = self.obstacle_turn_target_angles[i]
                
                angle_error = target_angle - theta
                angle_error = (angle_error + math.pi) % (2 * math.pi) - math.pi

                if abs(angle_error) < 0.1:
                    self.obstacle_states[i] = 'moving'
                    self.obstacle_turn_target_angles[i] = None
                    self.obstacle_turn_centers[i] = None
                else:
                    turn_speed = np.sign(angle_error) * abs(self.obstacle_speeds[i][1])
                    new_theta = theta + turn_speed * dt
                    new_x_r = x_c - self.d_center * math.cos(new_theta)
                    new_y_r = y_c - self.d_center * math.sin(new_theta)
                    self.obstacle_poses[i] = [new_x_r, new_y_r, new_theta]
            
            elif current_state == 'moving':
                x_r, y_r, theta = self.obstacle_poses[i]
                v, omega = self.obstacle_speeds[i]
                
                # 应用和主机器人完全相同的运动学模型
                new_x_r = x_r + (v * math.cos(theta) + self.d_center * omega * math.sin(theta)) * dt
                new_y_r = y_r + (v * math.sin(theta) - self.d_center * omega * math.cos(theta)) * dt
                
                # 碰撞检测用的新theta也应该提前计算
                new_theta_for_check = theta + omega * dt
                
                # 用更新后的位姿计算新的几何中心，用于碰撞检测
                new_x_c = new_x_r + self.d_center * math.cos(new_theta_for_check)
                new_y_c = new_y_r + self.d_center * math.sin(new_theta_for_check)

                is_colliding = False
                if not (-self.ENV_SIZE/2 + obs_bounding_radius < new_x_c < self.ENV_SIZE/2 - obs_bounding_radius and
                        -self.ENV_SIZE/2 + obs_bounding_radius < new_y_c < self.ENV_SIZE/2 - obs_bounding_radius):
                    is_colliding = True

                if not is_colliding:
                    for j in range(len(self.obstacle_poses)):
                        if i == j: continue
                        other_x_r, other_y_r, other_theta = self.obstacle_poses[j]
                        other_x_c = other_x_r + self.d_center * math.cos(other_theta)
                        other_y_c = other_y_r + self.d_center * math.sin(other_theta)
                        
                        if math.hypot(new_x_c - other_x_c, new_y_c - other_y_c) < 2 * obs_bounding_radius:
                            is_colliding = True
                            break
                
                if not is_colliding:
                    dist_to_pickup = math.hypot(new_x_c - self.pickup_location[0], new_y_c - self.pickup_location[1])
                    dist_to_dropoff = math.hypot(new_x_c - self.dropoff_location[0], new_y_c - self.dropoff_location[1])
                    
                    if dist_to_pickup < 0.3 or dist_to_dropoff < 0.3:
                        is_colliding = True

                if is_colliding:
                    self.obstacle_states[i] = 'turning'
                    self.obstacle_turn_target_angles[i] = theta + math.pi
                else:
                    # 之前是: new_theta = theta + omega * dt
                    # 现在直接使用上面计算碰撞时用的角度
                    new_theta = new_theta_for_check 
                    self.obstacle_poses[i] = [new_x_r, new_y_r, new_theta]
            
            patch_index = 4 + i * 2
            xy_obs_body, xy_obs_gripper = self.__calculate_model_vertices(self.obstacle_poses[i], self.ROBOT_SIZE, self.GRIPPER_SIZE)
            self.patches[patch_index].xy = xy_obs_body
            self.patches[patch_index + 1].xy = xy_obs_gripper

        self.figure.canvas.draw_idle()
        self.figure.canvas.flush_events()

    def set_mobile_base_speed_and_gripper_power(self, v, omega, gripper_power):
        delta_time = int(round(time.time()*1000)) - self.last_time_set_mobile_base_speed
        if delta_time > self.TIMEOUT_SET_MOBILE_BASE_SPEED:
            v, omega = self.__saturate_speeds(v, omega)
            dt_s = delta_time / 1000.0
            
            x_r, y_r, theta = self.robot_pose
            
            self.robot_pose[0] += (v * math.cos(theta) + self.d_center * omega * math.sin(theta)) * dt_s
            self.robot_pose[1] += (v * math.sin(theta) - self.d_center * omega * math.cos(theta)) * dt_s
            self.robot_pose[2] += omega * dt_s
            
            self.last_time_set_mobile_base_speed = int(round(time.time()*1000))
            self.__update_plot()

    def __saturate_speeds(self, v, omega):
        v = max(-self.MAX_LINEAR_SPEED, min(v, self.MAX_LINEAR_SPEED))
        omega = max(-self.MAX_ANGULAR_SPEED, min(omega, self.MAX_ANGULAR_SPEED))
        return v, omega
    
    def get_poses(self):
        while int(round(time.time()*1000)) - self.last_time_get_poses < self.TIMEOUT_GET_POSES: pass
        self.last_time_get_poses = int(round(time.time()*1000))
        return [self.robot_pose, self.pickup_location, self.dropoff_location, self.obstacle_poses]