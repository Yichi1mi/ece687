import math
import time
import numpy as np
import cvxpy as cp
from config import *

def clf_cbf_qp_controller(robot, goal, current_robot_pose, current_obstacle_poses):
    """
    Calculates robot wheel speeds using a CLF-CBF-QP based controller.
    This controller ensures the robot moves towards a goal while avoiding obstacles.
    """
    # === Unpack System State ===
    # Unpack the pose information passed into the function.
    x_r, y_r, theta = current_robot_pose
    gx, gy = goal[0], goal[1]

    # Calculate positions of the robot's geometric center and gripper.
    x_c = x_r + D_CENTER * math.cos(theta)
    y_c = y_r + D_CENTER * math.sin(theta)

    # === Define QP Variables ===
    # These are the variables the QP solver will optimize for.
    v = cp.Variable()
    omega = cp.Variable()
    delta = cp.Variable() 

    # === Define CLF (Control Lyapunov Function) ===
    # This function and its constraint drive the robot towards the goal.
    error_x = x_c - gx
    error_y = y_c - gy
    V = error_x**2 + error_y**2
    
    dot_V = 2 * (error_x * math.cos(theta) + error_y * math.sin(theta)) * v
    clf_constraint = dot_V + CLF_GAMMA * V <= delta

    # === Define CBF (Control Barrier Functions) ===
    # These functions and their constraints keep the robot away from obstacles.
    cbf_constraints = []
    safe_dist_sq = (ROBOT_WIDTH / 2)**2 + (ROBOT_LENGTH / 2)**2

    for obs_pose in current_obstacle_poses:
        if not obs_pose: continue
        ox_r, oy_r, o_theta = obs_pose
        ox_c = ox_r + D_CENTER * math.cos(o_theta)
        oy_c = oy_r + D_CENTER * math.sin(o_theta)
        
        h = (x_c - ox_c)**2 + (y_c - oy_c)**2 - safe_dist_sq - CBF_MARGIN
        h_dot = 2 * (x_c - ox_c) * (v * math.cos(theta)) + \
                2 * (y_c - oy_c) * (v * math.sin(theta))
        
        cbf_constraints.append(h_dot + CBF_ALPHA * h >= 0)

    # === Define Vortex Field for local obstacle perturbation ===
    # This adds a rotational component to help navigate around obstacles in close proximity.
    omega_bias = 0.0
    vec_to_goal_x = gx - x_c
    vec_to_goal_y = gy - y_c
    
    for obs_pose in current_obstacle_poses:
        if not obs_pose: continue
        ox_r, oy_r, o_theta = obs_pose
        ox_c = ox_r + D_CENTER * math.cos(o_theta)
        oy_c = oy_r + D_CENTER * math.sin(o_theta)
        
        vec_to_obs_x = ox_c - x_c
        vec_to_obs_y = oy_c - y_c
        dist_sq_to_obs = vec_to_obs_x**2 + vec_to_obs_y**2

        if dist_sq_to_obs < VORTEX_ACTIVATION_RADIUS**2:
            angle_to_obs = math.atan2(vec_to_obs_y, vec_to_obs_x)
            angle_diff = math.atan2(math.sin(angle_to_obs - theta), math.cos(angle_to_obs - theta))

            if abs(angle_diff) < math.radians(VORTEX_FIELD_OF_VIEW_DEG / 2.0):
                cross_product = vec_to_goal_x * vec_to_obs_y - vec_to_goal_y * vec_to_obs_x
                vortex_direction = -np.sign(cross_product) if cross_product != 0 else -1.0
                vortex_strength = VORTEX_STRENGTH_K / (dist_sq_to_obs + 1e-7)
                omega_bias += vortex_direction * vortex_strength


    # === Define QP Cost Function ===
    # The solver tries to minimize this cost, finding a balance between following a reference
    # controller and satisfying the CLF constraint.
    dist_to_goal = math.sqrt(V)
    v_ref = np.clip(REF_K_V * dist_to_goal, -MAX_LINEAR_SPEED, MAX_LINEAR_SPEED)

    angle_to_goal = math.atan2(gy - y_c, gx - x_c)
    angle_error = math.atan2(math.sin(angle_to_goal - theta), math.cos(angle_to_goal - theta))
    omega_ref = np.clip(REF_K_OMEGA * angle_error, -MAX_ANGULAR_SPEED, MAX_ANGULAR_SPEED)
    
    cost = cp.Minimize((v - v_ref)**2 + QP_P_OMEGA * (omega - (omega_ref + omega_bias))**2 + QP_P_DELTA * delta)

    # === Assemble and Solve the QP ===
    constraints = [
        v >= -MAX_LINEAR_SPEED, v <= MAX_LINEAR_SPEED,
        omega >= -MAX_ANGULAR_SPEED, omega <= MAX_ANGULAR_SPEED,
        delta >= 0,
        clf_constraint
    ] + cbf_constraints

    problem = cp.Problem(cost, constraints)
    
    try:
        problem.solve(solver=cp.OSQP, verbose=False)

        if problem.status in [cp.OPTIMAL, cp.OPTIMAL_INACCURATE]:
            v_sol = float(v.value)
            w_sol = float(omega.value)
            robot.set_mobile_base_speed_and_gripper_power(v_sol, w_sol, 0.0)
        else:
            print(f"QP failed with status: {problem.status}. Rotating towards goal.")
            robot.set_leds(*COLOR_RED)
            robot.set_mobile_base_speed_and_gripper_power(0, omega_ref, 0)
    except Exception as e:
        print(f"QP solver threw an exception: {e}")
        robot.set_leds(*COLOR_RED)
        robot.set_mobile_base_speed_and_gripper_power(0, 0, 0)