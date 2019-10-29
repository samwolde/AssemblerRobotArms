import math

P = 20.0
I = 2.0
D = 3.0

ROBOT_NAME = "wheely"
ROBOT_NAME_1 = "wheely"

MAIN_LENGTH = 0.3

ARM_1 = MAIN_LENGTH/2.0
ARM_2 = MAIN_LENGTH/2.0
ARM_BASE_TOP = MAIN_LENGTH/10.0
ARM_BASE = MAIN_LENGTH/20.0


RANGE_SAMPLES = 3

NODE_AVAILABLE = 0
NODE_WALL = 1
NODE_BUFFER = 2

INTERVAL = 0.32
ROW = COL = 125


# Number of steps the arm uses to move from current to final position 
ARM_STEP = 50

# Camera resolution
CAMERA_WIDTH = 800
CAMERA_HEIGHT = 800

# Arm angles limit
# (lower_limit, upper_limit)
ARM_ANGLE_LIMIT = [
    (-360, 360),    # Arm base
    (-130, 130),       # Arm base to arm1 joint
    (0, 180),    # Arm1 to arm2 joint
    (-180, 180)       # Gripper
]

ARM_BASE_INIT_HEIGHT = 0.2


# Gripper to object alignment vertical and horizontal step
HOR_STEP = 0.7
VER_STEP = MAIN_LENGTH/150

# Gripper angle step
GRIPPER_STEP = 0.4

# Retract step
STRAIGHT_ARM_STEP = 0.01 * 3

# Object detection rate
OBJ_DET_RATE = 40

# list of points in the house defining the different rooms
MAZE_ROOMS = [[2.78,0],[1.1,6.7]]