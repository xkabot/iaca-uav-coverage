import os
import sys


from shared_c import SharedConstants

class SupervisorConstants:
    """SupervisorConstants is a class that contains constants for the SupervisorController."""
    p_max = 220.0
    alpha_pheromone = 0.9
    lam = 0.8
    epsilon = 1e-30
    
    startup_hover_time = 3.0
    
    save_maps_interval = 100
    print_interval = 250
    
    supervisor_step_size = 15
    
    
    
    def __init__(self, shared: SharedConstants, config: dict={}):
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
        
        for key, value in config.items():
            if value is not None:
                setattr(self, key, value)
                
        self.rng = shared.rng
    