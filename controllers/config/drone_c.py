import math

from shared_c import SharedConstants

class DroneConstants:
    """Constants for the drone controller."""
    boundary_strength = 0.9
    boundary_margin = 10.0

    delta_v_max = 0.09
    alpha_velocity = 0.9
    max_world_speed = 1.5

    wind_update_period = 20.0
    wind_std = 0.03
    wind_max = 0.08

    d_max = math.sqrt(2.0)
    
    def __init__(self, shared: SharedConstants, config: dict={}):
        self.flying_altitude = shared.height_desired
        
        self.world_x_min = shared.world_x_min
        self.world_x_max = shared.world_x_max
        self.world_y_min = shared.world_y_min
        self.world_y_max = shared.world_y_max
        
        for key, value in config.items():
            if value is not None:
                setattr(self, key, value)

        self.rng = shared.rng
    
    
# BOUNDARY_STRENGTH = .9
# BOUNDARY_MARGIN = 10.0

# DELTA_V_MAX = 0.09
# ALPHA_VELOCITY = 0.9  # movement smoothing factor
# MAX_WORLD_SPEED = 1.5

# WIND_UPDATE_PERIOD = 20.0
# WIND_STD = 0.03
# WIND_MAX = 0.08

# FLYING_ATTITUDE = sc.HEIGHT_DESIRED

# D_MAX = math.sqrt(2.0)  # max diagonal distance between neighboring cells in grid units

# WORLD_X_MIN = sc.WORLD_X_MIN
# WORLD_X_MAX = sc.WORLD_X_MAX
# WORLD_Y_MIN = sc.WORLD_Y_MIN
# WORLD_Y_MAX = sc.WORLD_Y_MAX