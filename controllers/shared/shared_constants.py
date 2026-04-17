## GLOBAL VARIABLES

# Define a shared seed so changes can be compared
SEED = 1738

MAX_STEPS = 15000
HEIGHT_DESIRED = 2.0

# in meters from center
WORLD_X_MIN = -100.0
WORLD_X_MAX = 100.0
WORLD_Y_MIN = -100.0
WORLD_Y_MAX = 100.0

GRID_ROWS = 200
GRID_COLS = 200

SENSOR_RADIUS_METERS = 10

cell_size_x = (WORLD_X_MAX - WORLD_X_MIN) / (GRID_COLS - 1)
cell_size_y = (WORLD_Y_MAX - WORLD_Y_MIN) / (GRID_ROWS - 1)
AVG_CELL_SIZE = 0.5 * (cell_size_x + cell_size_y)

SENSOR_RADIUS_CELLS = max(1, round(SENSOR_RADIUS_METERS / AVG_CELL_SIZE))
