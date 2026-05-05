import os, sys
import numpy as np

current_dir = os.path.dirname(__file__)
shared_path = os.path.abspath(os.path.join(current_dir, "..", "shared"))
sys.path.append(shared_path)

import shared_constants as sc

SEED = sc.SEED
RNG = sc.RNG

WORLD_X_MIN = sc.WORLD_X_MIN
WORLD_X_MAX = sc.WORLD_X_MAX
WORLD_Y_MIN = sc.WORLD_Y_MIN
WORLD_Y_MAX = sc.WORLD_Y_MAX

GRID_ROWS = sc.GRID_ROWS
GRID_COLS = sc.GRID_COLS

P_MAX = 1_000_000.0
ALPHA_PHEROMONE = 0.9975  # Exponential decay factor for pheromone
LAMBDA = 0.87  # Spatial decay factor for drone pheromone
EPSILON = 1e-30

HEIGHT_DESIRED = sc.HEIGHT_DESIRED

STARTUP_HOVER_TIME = 3.0

SENSOR_RADIUS_CELLS = sc.SENSOR_RADIUS_CELLS
MAX_STEPS = sc.MAX_STEPS
SAVE_MAPS_INTERVAL = 100
PRINT_INTERVAL = 250

# Controls how often the supervisor runs its code to update pheromone n stuff. Paper used 15 (maybe 16; they said 500 / 32, which is NOT an even quotient)
SUPERVISOR_STEP_SIZE = 15

# Drone spawning parameters
SPAWN_RADIUS = 20.0
NUMBER_OF_DRONES = sc.NUMBER_OF_DRONES

# Exclusion zone parameters
USE_EXCLUSION = sc.USE_EXCLUSION
EXCLUSION_MARGIN_CELLS = 15


def make_exclusion_mask():
    # Use to create a boolean mask of shape (GRID_ROWS, GRID_COLS) where True indicates excluded areas, this is for manual creation,
    # but could very easily create like a visual creator if wanted.
    j_idx = np.arange(GRID_COLS)
    i_idx = np.arange(GRID_ROWS)
    X = WORLD_X_MIN + j_idx * (WORLD_X_MAX - WORLD_X_MIN) / (GRID_COLS - 1)
    Y = WORLD_Y_MIN + i_idx * (WORLD_Y_MAX - WORLD_Y_MIN) / (GRID_ROWS - 1)
    XX, YY = np.meshgrid(X, Y)  # shape (rows, cols)
    mask = np.abs(XX) + np.abs(YY) > 500
    return mask
