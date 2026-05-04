import numpy as np


class SharedConstants:
    """Contains constants that are shared across both the supervisor and drones."""
    # Seed used for simulation
    seed = 20000
    
    # Shared time constants for algorithm
    tick_rate_ms = 32  # Each Webots simulation step is 32ms
    tick_rate_sec = tick_rate_ms / 1000  # Seconds per simulation step
    max_steps = 750
    
    # Shared grid constants
    # Min and max is in meters relative to center of grid
    world_x_min = -450.0
    world_x_max = 450.0
    world_y_min = -450.0
    world_y_max = 450.0
    
    grid_rows = 500
    grid_cols = 500
    
    cell_size_x = (world_x_max - world_x_min) / (grid_cols - 1)
    cell_size_y = (world_y_max - world_y_min) / (grid_rows - 1)
    avg_cell_size = 0.5 * (cell_size_x + cell_size_y)
    
    # Shared drone constants
    number_of_drones = 2 
    sensor_radius_meters = 10
    sensor_radius_cells = max(1, round(sensor_radius_meters / avg_cell_size))
    height_desired = 2.0
    
    # Set true if you want to use a custom search area (non-rectangular). Drones will avoid any area included in the exclusion bitmap created in supervisor_constants.
    use_exclusion = False                            
    
    
    def __init__(self, config: dict={}, rng=None):
        # Set the values contained in a dictionary if provided
        for key, value in config.items():
            if value is not None:
                setattr(self, key, value)
        
        # If a numpy rng generator is not provided, initialize one
        if rng is None:
            self.rng = np.random.default_rng(self.seed)
        else:
            self.rng = rng
                