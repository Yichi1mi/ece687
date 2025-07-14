import time
import math
from mobile_manipulator_unicycle import MobileManipulatorUnicycle

# --- 1. 常量和设置 ---

# 控制器增益 (可以根据机器人实际表现进行调整)
K_V = 0.4  # 线性速度的比例增益
K_OMEGA = 1.5 # 角速度的比例增益

# 阈值
DISTANCE_THRESHOLD = 0.05  # 到达目标的距离阈值 (单位: 米)
ANGLE_THRESHOLD = 0.1      # 对准目标的角度阈值 (单位: 弧度)

# 机器人初始化
# 请确保 robot_id 和 backend_server_ip 与您的设置匹配
robot = MobileManipulatorUnicycle(robot_id=2, backend_server_ip="192.168.0.2")


# --- 2. 等待获取位姿信息 ---

print("正在等待位姿数据...")
# 从机器人获取位姿元组 (robot_pose, pickup_pose, dropoff_pose, obstacles_poses)
robot_pose, pickup_pose, _, _ = robot.get_poses()

# 循环等待，直到成功获取到机器人位姿和目标位姿
while not robot_pose or not pickup_pose:
    time.sleep(0.5)
    robot_pose, pickup_pose, _, _ = robot.get_poses()
    print("...")

print(f"成功获取位姿!")
print(f"机器人初始位姿 (pose0): {robot_pose}")
print(f"目标抓取位姿 (pose1): {pickup_pose}")


# --- 3. 导航到目标点 (自动控制循环) ---

# 设定目标坐标
target_x = pickup_pose[0]
target_y = pickup_pose[1]

print("\n开始自动导航前往目标点...")
while True:
    # 实时获取机器人当前位姿
    robot_pose, _, _, _ = robot.get_poses()
    if not robot_pose:
        continue  # 如果位姿数据无效，则跳过此次循环

    current_x = robot_pose[0]
    current_y = robot_pose[1]
    current_theta = robot_pose[2]

    # --- 计算误差 ---
    # 计算机器人与目标的直线距离
    distance_error = math.sqrt((target_x - current_x)**2 + (target_y - current_y)**2)-0.4
    
    # 检查是否已到达目标
    if distance_error < DISTANCE_THRESHOLD:
        print("已到达目标点!")
        robot.set_mobile_base_speed_and_gripper_power(v=0.0, omega=0.0, gripper_power=0.0)
        break # 退出导航循环

    # --- P控制器计算 ---
    # 计算机器人朝向目标的期望角度
    desired_angle = math.atan2(target_y - current_y, target_x - current_x)
    # 计算角度误差
    angle_error = desired_angle - current_theta
    # 将角度误差归一化到 [-π, π] 区间，确保机器人选择最短路径旋转
    angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))

    # --- 应用控制指令 ---
    # 如果角度偏差较大，优先原地旋转对准目标
    if abs(angle_error) > ANGLE_THRESHOLD:
        v = 0.0 # 旋转时前进速度为0
        omega = K_OMEGA * angle_error
    else:
        # 如果已对准目标，则开始前进，同时微调角度
        v = K_V * distance_error
        omega = K_OMEGA * angle_error
    
    # 限制最大速度，防止机器人失控
    v = max(min(v, 0.2), -0.2)      # 限制最大前进速度为 0.2 m/s
    omega = max(min(omega, 1.0), -1.0) # 限制最大旋转速度为 1.0 rad/s
    
    # 发送控制指令给机器人
    print(f"距离目标: {distance_error:.2f} m, 角度误差: {angle_error:.2f} rad  ==>  V: {v:.2f}, W: {omega:.2f}")
    robot.set_leds(0, 255, 0) # 移动时亮绿灯
    robot.set_mobile_base_speed_and_gripper_power(v, omega, gripper_power=0.0) # 移动时夹爪不动
    
    time.sleep(0.1) # 短暂延时，给机器人反应时间


# --- 4. 执行抓取动作 ---
# test.py (MODIFIED)
import time
from mobile_manipulator_unicycle import MobileManipulatorUnicycle
# Import the new, high-level task function
from avoid_obstacle import run_full_task_with_dwa

# --- 1. Robot Initialization ---
# Make sure robot_id and IP are correct for your setup
robot = MobileManipulatorUnicycle(robot_id=3, backend_server_ip="192.168.0.2")

# --- 2. Wait for Poses ---
print("Initializing... Waiting for the first valid pose data from the robot.")
while True:
    robot_pose, pickup, dropoff, obstacles = robot.get_poses()
    if all([robot_pose, pickup, dropoff]):
        print("Successfully received initial poses. Starting the task.")
        break
    time.sleep(0.5)

# --- 3. Run the Full Task ---
# This single function now handles all navigation and manipulation
run_full_task_with_dwa(robot)

# --- 4. Shutdown ---
print("\nFull task sequence has been completed.")
# Stop the robot and turn off LEDs
robot.set_mobile_base_speed_and_gripper_power(0., 0., 0.)
robot.set_leds(0, 0, 0)
print("\n开始执行抓取动作...")
robot.set_leds(0, 0, 255) # 操作时亮蓝灯

# 第1步: 移动机械臂到预备抓取位置 (此处的坐标为示例)
print("步骤1: 移动机械臂至预备位置...")
robot.set_arm_pose(100, 50) # 此函数默认会阻塞等待机械臂到位

# 第2步: 张开夹爪
print("步骤2: 张开夹爪...")
robot.set_mobile_base_speed_and_gripper_power(v=0.0, omega=0.0, gripper_power=1.0)
time.sleep(2.0) # 等待夹爪完全张开

# 第3步: 移动机械臂下降进行抓取 (此处的坐标为示例)
print("步骤3: 下降机械臂...")
robot.set_arm_pose(200, -60) 

# 第4步: 闭合夹爪
print("步骤4: 闭合夹爪...")
start_time = time.time()
while time.time() - start_time < 2.:
    robot.set_mobile_base_speed_and_gripper_power(v=0.0, omega=0.0, gripper_power=-1.0)
    robot.set_leds(0, 0, 255)
    time.sleep(0.05)

# 第5步: 抬起物体
print("步骤5: 抬起物体...")
robot.set_arm_pose(100, 50)

print("RForward for 2 seconds and set the LEDs green")
start_time = time.time()
while time.time() - start_time < 1.5:
    robot.set_mobile_base_speed_and_gripper_power(v=-0.1, omega=0.0, gripper_power=0.0)
    robot.set_leds(0, 255, 0)
    
    time.sleep(0.05)
    
print("\n开始执行抓取动作...")
robot.set_leds(0, 0, 255) # 操作时亮蓝灯

print("步骤2: 张开夹爪...")
robot.set_mobile_base_speed_and_gripper_power(v=0.0, omega=0.0, gripper_power=1.0)
time.sleep(2.0) # 等待夹爪完全张开

print("\n任务完成!")
final_pose, _, _, _ = robot.get_poses()
print(f"机器人最终位姿: {final_pose}")

# --- 5. 任务结束 ---
# 停止机器人所有动作并熄灭LED
robot.set_mobile_base_speed_and_gripper_power(0., 0., 0.)
robot.set_leds(0, 0, 0)