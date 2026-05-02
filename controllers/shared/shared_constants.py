## GLOBAL VARIABLES
import numpy as np

# Define a shared seed so changes can be compared
SEED = 2000
RNG = np.random.default_rng(SEED)

TICK_RATE_MS = 32  # each webots simulation step is 32 ms
TICK_RATE_SEC = TICK_RATE_MS / 1000  # seconds per simulation step

MAX_STEPS = 100000
HEIGHT_DESIRED = 2.0

# in meters from center
WORLD_X_MIN = -450.0
WORLD_X_MAX = 450.0
WORLD_Y_MIN = -450.0
WORLD_Y_MAX = 450.0

GRID_ROWS = 500
GRID_COLS = 500

SENSOR_RADIUS_METERS = 10

cell_size_x = (WORLD_X_MAX - WORLD_X_MIN) / (GRID_COLS - 1)
cell_size_y = (WORLD_Y_MAX - WORLD_Y_MIN) / (GRID_ROWS - 1)
AVG_CELL_SIZE = 0.5 * (cell_size_x + cell_size_y)

SENSOR_RADIUS_CELLS = max(1, round(SENSOR_RADIUS_METERS / AVG_CELL_SIZE))

NUMBER_OF_DRONES = 4

# Set true if you want to use a custom search area (non-rectangular). Drones will avoid any area included in the exclusion bitmap created in supervisor_constants.
USE_EXCLUSION = False
