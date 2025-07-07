# config.py

# --- 机器人与环境尺寸 ---
ROBOT_WIDTH = 0.24
ROBOT_LENGTH = 0.32
OBSTACLE_RADIUS = ROBOT_LENGTH * 0.6  # 基于机器人尺寸的障碍物半径
ROBOT_BOUNDING_RADIUS = ROBOT_LENGTH * 0.6  # DWA中使用的机器人包围半径

# --- 导航与状态机参数 ---
APPROACH_DISTANCE = 0.4      # 预备点距离目标的距离
NO_GO_ZONE_RADIUS = 0.35      # 禁行区的半径 (与预备点距离一致)
SAFE_DIST_FROM_OBSTACLE = 0.36 # 预备点距离障碍物的最小安全距离
FINAL_APPROACH_SPEED = 0.05    # 最后直线靠近的速度
RETREAT_SPEED = -0.05          # 撤退速度

# --- DWA 规划器参数 ---
PREDICT_TIME = 1.0           # DWA 预测时间
GOAL_WEIGHT = 1.0            # DWA 评分权重：目标距离
OBSTACLE_WEIGHT = 1.5        # DWA 评分权重：障碍物距离
HEADING_WEIGHT = 1.0         # DWA 评分权重：朝向
REVERSE_PENALTY = 2.0        # DWA 评分权重：倒车惩罚
