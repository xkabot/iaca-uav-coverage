import os
import sys

import numpy as np

current_dir = os.path.dirname(__file__)
shared_path = os.path.abspath(os.path.join(current_dir, "..", "shared"))
sys.path.append(shared_path)

from shared_c import SharedConstants


class SupervisorConstants:
    """Contains constants used by the supervisor controller."""
    p_max = 1_000_000.0
    alpha_pheromone = 0.9975  # Exponential decay factor for pheromone
    lam = 0.87  # Spatial decay factor for drone pheromone
    epsilon = 1e-30
    
    startup_hover_time = 3.0
    
    save_maps_interval = 100
    print_interval = 250
    
    # Controls how often the supervisor runs its code to update pheromone n stuff. Paper used 15 (maybe 16; they said 500 / 32, which is NOT an even quotient)
    supervisor_step_size = 15
    
    # Drone spawning parameters
    spawn_radius = 20.0
    
    # Local exclusion zone parameters
    exclusion_mask = None
    exclusion_margin_cells = 15 
    
    
    def __init__(self, shared: SharedConstants, config: dict={}):
        # Shared grid constants
        self.world_x_min = shared.world_x_min
        self.world_x_max = shared.world_x_max
        self.world_y_min = shared.world_y_min
        self.world_y_max = shared.world_y_max
        
        self.grid_rows = shared.grid_rows
        self.grid_cols = shared.grid_cols
        
        # Shared drone constants
        self.sensor_radius_cells = shared.sensor_radius_cells
        self.height_desired = shared.height_desired
        
        # Shared time steps constant
        self.max_steps = shared.max_steps
        
        # Shared drone spawning parameters
        self.number_of_drones = shared.number_of_drones
        
        # Shared exclusion zone parameters
        self.use_exclusion = shared.use_exclusion
        
        # Reassign any values provided by dictionary
        for key, value in config.items():
            if value is not None:
                setattr(self, key, value)
                
        # Reference the rng instance used by all instances in a simulation
        self.rng = shared.rng
    
        # Use to create a boolean mask of shape (grid_rows, grid_cols) where True indicates excluded areas, this is for manual creation,
        # but could very easily create like a visual creator if wanted.
        if self.use_exclusion:
            j_idx = np.arange(self.grid_cols)
            i_idx = np.arange(self.grid_rows)
            X = self.world_x_min + j_idx * (self.world_x_max - self.world_x_min) / (self.grid_cols - 1)
            Y = self.world_y_min + i_idx * (self.world_y_max - self.world_y_min) / (self.grid_rows - 1)
            XX, YY = np.meshgrid(X, Y)  # shape (rows, cols)
            
            self.exclusion_mask = np.abs(XX) + np.abs(YY) > 500
        