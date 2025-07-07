import math
import time
import numpy as np

def dwa_planner(robot, goal):
    dt = 0.05
    predict_time = 1.0
    v_samples = np.linspace(-0.05, 0.1, 15)
    w_samples = np.radians(np.linspace(-40, 40, 21))

    robot_pose, _, _, obstacle_poses, _ = robot.get_poses()
    best_score = -float("inf")
    best_v = 0
    best_w = 0

    for v in v_samples:
        for w in w_samples:
            traj = simulate_trajectory(robot_pose, v, w, dt, predict_time)
            score = evaluate_trajectory(robot, traj, goal, obstacle_poses, v, w)
            if score > best_score:
                best_score = score
                best_v = v
                best_w = w

    robot.set_mobile_base_speed_and_gripper_power(best_v, best_w, 0)
    time.sleep(0.05)


def simulate_trajectory(start_pose, v, w, dt, predict_time):
    x, y, theta = start_pose
    traj = []
    for _ in range(int(predict_time / dt)):
        x += v * math.cos(theta) * dt
        y += v * math.sin(theta) * dt
        theta += w * dt
        traj.append([x, y, theta])
    return traj


def evaluate_trajectory(robot, robot_traj, goal, obstacle_poses, v, w):
    
    ROBOT_BOUNDING_RADIUS = 0.22 
    OBSTACLE_RADIUS = max(robot.ROBOT_SIZE) * 0.5
    SAFE_DIST = ROBOT_BOUNDING_RADIUS + OBSTACLE_RADIUS + 0.03

    goal_weight = 1.0
    obs_weight = 1.5
    heading_weight = 1.0
    reverse_penalty = 2.0

    final_x, final_y, final_theta = robot_traj[-1]

    # 1. 目标分：离目标越近越好
    score_goal = -math.hypot(final_x - goal[0], final_y - goal[1])
    
    # 2. 朝向分：角度误差越小越好
    target_angle = math.atan2(goal[1] - final_y, goal[0] - final_x)
    heading_error = abs((target_angle - final_theta + math.pi) % (2 * math.pi) - math.pi)
    score_heading = -heading_error
    
    # 3. 倒车惩罚分
    score_reverse = v * reverse_penalty if v < 0 else 0
    
    # --- 碰撞检测 ---
    score_obs = 0
    
    robot_center_offset = robot.ROBOT_SIZE[1] / 2.0
    final_robot_center_x = final_x + robot_center_offset * math.cos(final_theta)
    final_robot_center_y = final_y + robot_center_offset * math.sin(final_theta)

    for obs_pose in obstacle_poses:
        dist = math.hypot(final_robot_center_x - obs_pose[0], final_robot_center_y - obs_pose[1])
        if dist < SAFE_DIST:
            return -float("inf")
        score_obs += dist
        
    if obstacle_poses:
        score_obs /= len(obstacle_poses)

    # --- 最终总分 ---
    return (goal_weight * score_goal +
            obs_weight * score_obs +
            heading_weight * score_heading +
            score_reverse)