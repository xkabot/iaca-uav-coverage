import os, sys

current_dir = os.path.dirname(__file__)
shared_path = os.path.abspath(os.path.join(current_dir, "..", "shared"))
sys.path.append(shared_path)

import shared_constants as sc

SEED = sc.SEED

WORLD_X_MIN = sc.WORLD_X_MIN
WORLD_X_MAX = sc.WORLD_X_MAX
WORLD_Y_MIN = sc.WORLD_Y_MIN
WORLD_Y_MAX = sc.WORLD_Y_MAX

GRID_ROWS = sc.GRID_ROWS
GRID_COLS = sc.GRID_COLS

P_MAX = 220.0
ALPHA_PHEROMONE = 0.99
LAMBDA = 0.9
EPSILON = 1e-30

HEIGHT_DESIRED = sc.HEIGHT_DESIRED

STARTUP_HOVER_TIME = 3.0

SENSOR_RADIUS_CELLS = sc.SENSOR_RADIUS_CELLS
MAX_STEPS = sc.MAX_STEPS
SAVE_INTERVAL = 25
PRINT_INTERVAL = 250

# Controls how often the supervisor runs its code to update pheromone n stuff. Paper used 15 (maybe 16; they said 500 / 32, which is NOT an even quotient)
SUPERVISOR_STEP_SIZE = 15
