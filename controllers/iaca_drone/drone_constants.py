import os, sys, math

current_dir = os.path.dirname(__file__)
shared_path = os.path.abspath(os.path.join(current_dir, "..", "shared"))
sys.path.append(shared_path)

import shared_constants as sc

SEED = sc.SEED
RNG = sc.RNG

BOUNDARY_STRENGTH = .9
BOUNDARY_MARGIN = 10.0

EXCLUSION_STRENGTH = 1.5
EXCLUSION_MARGIN = 20

DELTA_V_MAX = 0.09
ALPHA_VELOCITY = 0.9  # movement smoothing factor
MAX_WORLD_SPEED = 1.5

WIND_UPDATE_PERIOD = 20.0
WIND_STD = 0.03
WIND_MAX = 0.08

FLYING_ATTITUDE = sc.HEIGHT_DESIRED

D_MAX = math.sqrt(2.0)  # max diagonal distance between neighboring cells in grid units

WORLD_X_MIN = sc.WORLD_X_MIN
WORLD_X_MAX = sc.WORLD_X_MAX
WORLD_Y_MIN = sc.WORLD_Y_MIN
WORLD_Y_MAX = sc.WORLD_Y_MAX
