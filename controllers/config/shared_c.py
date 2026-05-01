import numpy as np


class SharedConstants:
    """Contains constants that are shared across both the supervisor and drones."""
    seed = 2000
    
    tick_rate_ms = 32  # Each Webots simulation step is 32ms
    tick_rate_sec = tick_rate_ms / 1000  # Seconds per simulation step
    
    max_steps = 100000
    height_desired = 2.0
    
    # In meters from the center of the grid
    world_x_min = -350.0
    world_x_max = 350.0
    world_y_min = -350.0
    world_y_max = 350.0
    
    grid_rows = 500
    grid_cols = 500
    
    sensor_radius_meters = 10
    
    cell_size_x = (world_x_max - world_x_min) / (grid_cols - 1)
    cell_size_y = (world_y_max - world_y_min) / (grid_rows - 1)
    avg_cell_size = 0.5 * (cell_size_x + cell_size_y)
    
    sensor_radius_cells = max(1, round(sensor_radius_meters / avg_cell_size))
    
    number_of_drones = 4                                   
    
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
                