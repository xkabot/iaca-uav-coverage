import os
import sys

import numpy as np

current_dir = os.path.dirname(__file__)
shared_path = os.path.abspath(os.path.join(current_dir, "..", "shared"))
sys.path.append(shared_path)

from shared_c import SharedConstants, valid_parameter


class SupervisorConstants:
    """Contains constants used by the supervisor controller."""
    
    name = "supervisor"

    # Supervisor parameters
    p_max = 1_000_000.0  # Max pheromone value
    alpha_pheromone = 0.9975  # Exponential decay factor for pheromone
    noise_fraction = 0.05  # Random noise for pheromone updates
    lam = 0.87  # Spatial decay factor for drone pheromone
    epsilon = 1e-30
    priority_exponent = 1.0  # Paper says something about setting it to 4.0

    # Data interval parameters
    save_maps_interval = 100
    print_interval = 250

    # Controls how often the supervisor runs its code to update pheromone n stuff. Paper used 15 (maybe 16; they said 500 / 32, which is NOT an even quotient)
    supervisor_step_size = 15

    # Drone spawning parameters
    spawn_radius = 20.0
    startup_hover_time = 3.0

    # Local exclusion zone parameters
    use_exclusion = False
    exclusion_margin_cells = 15

    def __init__(self, shared: SharedConstants, config: dict = {}):
        """Initializes the algorithm's supervisor constants from a dictionary
        of key and value pairs, and pre-initalized globally shared constants.

        If a dictionary is not passed or a value is of a mistmatching type, the default values will be used.
        Additionally, a key will be ignored if it is not a valid parameter in this class.

        Args:
            shared (SharedConstants): An instance with global parameters used in the algorithm.
            config (dict, optional): A dictionary containing configuration parameters where keys
                match class attribute names and values are the desired settings. Defaults to {}.
        """

        # Shared grid constants
        self.world_x_min = shared.world_x_min
        self.world_x_max = shared.world_x_max
        self.world_y_min = shared.world_y_min
        self.world_y_max = shared.world_y_max

        self.grid_rows = shared.grid_rows
        self.grid_cols = shared.grid_cols
        
        # Use to create a boolean mask of shape (grid_rows, grid_cols) where True indicates excluded areas, this is for manual creation,
        # but could very easily create like a visual creator if wanted.
        self.use_exclusion = shared.use_exclusion
        if self.use_exclusion:
            self.exclusion_mask = make_exclusion_mask(vars(self))
        else:
            self.exclusion_mask = None

        # Shared drone constants
        self.sensor_radius_cells = shared.sensor_radius_cells
        self.height_desired = shared.height_desired

        # Shared time steps constant
        self.max_steps = shared.max_steps

        # Shared drone spawning parameters
        self.number_of_drones = shared.number_of_drones

        # Reassign any values provided by dictionary
        for key, value in config.items():
            if value is not None:
                if not valid_parameter(self, key, value, self.name):
                    continue

                setattr(self, key, value)

        # Reference the rng instance used by all instances in a simulation
        self.rng = shared.rng
            
            
def make_exclusion_mask(data: dict) -> np.ndarray:
    """Construct a boolean NumPy array containing cells that prohibit a drone from entering."""
    j_idx = np.arange(data["grid_cols"])
    i_idx = np.arange(data["grid_rows"])
    
    X = data["world_x_min"] + j_idx * (data["world_x_max"] - data["world_x_min"]) / (
        data["grid_cols"] - 1
    )
    Y = data["world_y_min"] + i_idx * (data["world_y_max"] - data["world_y_min"]) / (
        data["grid_rows"] - 1
    )
    XX, YY = np.meshgrid(X, Y)  # shape (rows, cols)

    return np.abs(XX) + np.abs(YY) > 500
