import math

# === ROBOT PHYSICAL PARAMETERS ===
# Defines the physical dimensions and key points of the robot.
ROBOT_WIDTH = 0.24
ROBOT_LENGTH = 0.32
D_CENTER = ROBOT_LENGTH / 2.0  # Distance from rear axle to geometric center.

# === ROBOT OPERATIONAL LIMITS ===
# Maximum speeds for the mobile base.
MAX_LINEAR_SPEED = 0.22
MAX_ANGULAR_SPEED = 1.8

# === STATE MACHINE BEHAVIORS ===
# Parameters for specific autonomous actions.
BACKUP_TIME = 2.0
BACKUP_SPEED = -0.08
APPROACH_SPEED = 0.05 # Max speed during the FINAL_APPROACH phase.

# === FINAL APPROACH PARAMETERS ===
# Defines the final, high-precision docking phase.
GRIPPER_DISTANCE = 0.45          # (m) The desired final distance from geometric center to goal.
FINAL_APPROACH_DISTANCE = 0.6    # (m) Distance to switch from NAVIGATE to FINAL_APPROACH.
FINAL_DISTANCE_TOLERANCE = 0.01  # (m) Allowed final distance error.
FINAL_ANGLE_TOLERANCE = 0.02     # (rad) Allowed final angle error (approx 1 degree).

# === P-CONTROLLER GAINS for FINAL_APPROACH ===
# Proportional gains for the low-speed docking controller.
KP_LINEAR = 0.4
KP_ANGULAR = 0.8

# === QP CONTROLLER GAINS & PARAMETERS (for NAVIGATE phase) ===
# Tuning parameters for the main CLF-CBF QP navigation controller.
QP_P_OMEGA = 1.0
QP_P_DELTA = 100.0
REF_K_V = 0.5
REF_K_OMEGA = 1.0
CLF_GAMMA = 1.0
CBF_ALPHA = 1.0
CBF_MARGIN = 0.01

# === VORTEX FIELD PARAMETERS (for QP NAVIGATE phase) ===
# Parameters for the obstacle avoidance vortex field.
VORTEX_ACTIVATION_RADIUS = 0.6
VORTEX_FIELD_OF_VIEW_DEG = 180
VORTEX_STRENGTH_K = 100.0

# === LED COLOR DEFINITIONS (R, G, B) ===
# Standard color codes for robot status indication.
COLOR_RED = (255, 0, 0)
COLOR_YELLOW = (255, 255, 0)
COLOR_BLUE = (0, 0, 255)
COLOR_GREEN = (0, 255, 0)
COLOR_WHITE = (255, 255, 255)