from shared_c import SharedConstants

class SupervisorConstants:
    """Contains constants used by the supervisor controller."""
    p_max = 220.0
    alpha_pheromone = 0.9  # Exponential decay factor for pheromone
    lam = 0.8  # Spatial decay factor for drone pheromone
    epsilon = 1e-30
    
    startup_hover_time = 3.0
    
    save_maps_interval = 100
    print_interval = 250
    
    # Controls how often the supervisor runs its code to update pheromone n stuff. Paper used 15 (maybe 16; they said 500 / 32, which is NOT an even quotient)
    supervisor_step_size = 15
    
    
    def __init__(self, shared: SharedConstants, config: dict={}):
        # Set the shared constants
        self.world_x_min = shared.world_x_min
        self.world_x_max = shared.world_x_max
        self.world_y_min = shared.world_y_min
        self.world_y_max = shared.world_y_max
        
        self.grid_rows = shared.grid_rows
        self.grid_cols = shared.grid_cols
        
        self.height_desired = shared.height_desired
        
        self.sensor_radius_cells = shared.sensor_radius_cells
        self.max_steps = shared.max_steps
        
        self.number_of_drones = shared.number_of_drones
        
        # Reassign any values provided by dictionary
        for key, value in config.items():
            if value is not None:
                setattr(self, key, value)
                
        # Reference the rng instance used by all instances in a simulation
        self.rng = shared.rng
    