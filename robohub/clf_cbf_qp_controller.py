import math
import time
import numpy as np
import cvxpy as cp

def clf_cbf_qp_controller(robot, goal):

    # === Get System State ===
    robot_pose, _, _, obstacle_poses = robot.get_poses()
    x_r, y_r, theta = robot_pose

    robot_length = robot.ROBOT_SIZE[1]
    d_center = robot_length / 2.0
    d_gripper = 0.4

    # Position of geometric center
    x_c = x_r + d_center * math.cos(theta)
    y_c = y_r + d_center * math.sin(theta)
    
    # Position of the gripper
    x_g = x_r + d_gripper * math.cos(theta)
    y_g = y_r + d_gripper * math.sin(theta)

    # Goal position
    gx, gy = goal[0], goal[1]

    # === Control Variables for QP ===
    v = cp.Variable()
    omega = cp.Variable()
    delta = cp.Variable(nonneg=True) 

    # Physical limits
    v_max = robot.MAX_LINEAR_SPEED
    omega_max = robot.MAX_ANGULAR_SPEED

    # === CLF ===

    error_x = x_c - gx
    error_y = y_c - gy 
    V = error_x**2 + error_y**2

    gamma = 1.0
    
    theta_d = np.arctan2(-error_y, -error_x)
    e_theta = theta - theta_d
    e_theta = np.arctan2(np.sin(e_theta), np.cos(e_theta))
    V_theta = 0.5 * e_theta**2

    dot_V_pos = 2 * (error_x * math.cos(theta) + error_y * math.sin(theta)) * v
    dot_V_ori = e_theta * omega
    clf_constraint = dot_V_pos + dot_V_ori + gamma * (V + V_theta) <= delta

    # === CBF ===
    cbf_constraints = []
    margin = 0.01
    safe_dist_sq = (robot.ROBOT_SIZE[0] + margin)**2 + (robot.ROBOT_SIZE[1] + margin)**2
    VORTEX_ACTIVATION_RADIUS = 0.7

    for obs_pose in obstacle_poses:
        ox_r, oy_r, o_theta = obs_pose

        ox_c = ox_r + d_center * math.cos(o_theta)
        oy_c = oy_r + d_center * math.sin(o_theta)
        
        h = (x_c - ox_c)**2 + (y_c - oy_c)**2 - safe_dist_sq
        
        alpha = 1.0 
        h_dot = 2 * (x_c - ox_c) * (v * math.cos(theta)) + 2 * (y_c - oy_c) * (v * math.sin(theta))
        
        cbf_constraints.append(h_dot + alpha * h >= 0)
        
    omega_bias = 0.0
    k_vortex = 100
    epsilon = 1e-7
    
    VORTEX_ACTIVATION_RADIUS = 0.6
    VORTEX_FIELD_OF_VIEW_DEG = 180

    vec_to_goal_x = gx - x_c
    vec_to_goal_y = gy - y_c
    
    for obs_pose in obstacle_poses:
        ox_r, oy_r, o_theta = obs_pose
        ox_c = ox_r + d_center * math.cos(o_theta)
        oy_c = oy_r + d_center * math.sin(o_theta)
        
        vec_to_obs_x = ox_c - x_c
        vec_to_obs_y = oy_c - y_c
        
        dist_sq_to_obs = vec_to_obs_x**2 + vec_to_obs_y**2

        if dist_sq_to_obs < VORTEX_ACTIVATION_RADIUS**2:
            
            angle_to_obs = math.atan2(vec_to_obs_y, vec_to_obs_x)
            angle_diff = angle_to_obs - theta
            angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))

            if abs(angle_diff) < math.radians(VORTEX_FIELD_OF_VIEW_DEG / 2.0):
                
                cross_product = vec_to_goal_x * vec_to_obs_y - vec_to_goal_y * vec_to_obs_x
                
                vortex_direction = -np.sign(cross_product)
                vortex_strength = k_vortex / (dist_sq_to_obs + epsilon)
                
                omega_bias += vortex_direction * vortex_strength

    # === QP Cost Function ===
    
    # Reference P-controller
    dist_to_goal = math.sqrt(V)
    k_v = 0.5 
    v_ref = np.clip(k_v * dist_to_goal, -v_max, v_max)

    angle_to_goal = math.atan2(gy - y_c, gx - x_c)
    angle_error = (angle_to_goal - theta + math.pi) % (2 * math.pi) - math.pi
    k_omega = 1.0
    omega_ref = np.clip(k_omega * angle_error, -omega_max, omega_max)
    
    p_omega = 1.0
    p_delta = 100 
    cost = cp.Minimize((v - v_ref)**2 + p_omega * (omega - (omega_ref + omega_bias))**2 + p_delta * delta**2)

    constraints = [
        v >= -v_max, v <= v_max,
        omega >= -omega_max, omega <= omega_max,
        delta >= 0,
        clf_constraint
    ] + cbf_constraints

    problem = cp.Problem(cost, constraints)
    
    try:
        problem.solve(solver=cp.OSQP, verbose=False)

        if problem.status == cp.OPTIMAL or problem.status == cp.OPTIMAL_INACCURATE:
            v_val = v.value
            omega_val = omega.value
            print(v_val,omega_val)
            robot.set_mobile_base_speed_and_gripper_power(v_val, omega_val, 0)
        else:
            print(f"QP failed with status: {problem.status}. Rotating towards goal.")
            robot.set_mobile_base_speed_and_gripper_power(0, omega_ref, 0)
    except Exception as e:
        print(f"QP solver threw an exception: {e}")
        robot.set_mobile_base_speed_and_gripper_power(0, 0, 0)
