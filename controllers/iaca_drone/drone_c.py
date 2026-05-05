import math
import os
import sys


current_dir = os.path.dirname(__file__)
shared_path = os.path.abspath(os.path.join(current_dir, "..", "shared"))
sys.path.append(shared_path)

from shared_c import SharedConstants, valid_parameter


class DroneConstants:
    """Contains constants used by the drone controller."""

    name = "drone"

    boundary_strength = 0.9
    boundary_margin = 10.0

    delta_v_max = 0.09
    d_max = None
    alpha_velocity = 0.9  # Movement smoothing factor
    max_world_speed = 1.5

    wind_update_period = 20.0
    wind_std = 0.03
    wind_max = 0.08

    exclusion_strength = 1.5
    exclusion_margin = 20
    

    def __init__(self, shared: SharedConstants, config: dict = {}):
        """Initializes the algorithm's drone constants from a dictionary
        of key and value pairs, and pre-initalized globally shared constants.

        If a dictionary is not passed or a value is of a mistmatching type, the default values will be used.
        Additionally, a key will be ignored if it is not a valid parameter in this class.

        Args:
            shared (SharedConstants): An instance with global parameters used in the algorithm.
            config (dict, optional): A dictionary containing configuration parameters where keys
                match class attribute names and values are the desired settings. Defaults to {}.
        """
        
        # Set the shared constants
        self.flying_altitude = shared.height_desired

        # Set the shared world constants
        self.world_x_min = shared.world_x_min
        self.world_x_max = shared.world_x_max
        self.world_y_min = shared.world_y_min
        self.world_y_max = shared.world_y_max

        # Reassign any values provided by dictionary
        for key, value in config.items():
            if value is not None and key != "d_max":
                if not valid_parameter(self, key, value, self.name):
                    continue
                
                setattr(self, key, value)
                
        # Calculate max diagonal distance between neighboring cells in grid units if not provided
        if getattr(self, "d_max") is None:
            self.d_max = math.sqrt(shared.cell_size_x ** 2 + shared.cell_size_y ** 2)
        else:
            self.dmax = math.sqrt(2.0)

        # Reference the rng instance used by all instances in a simulation
        self.rng = shared.rng
