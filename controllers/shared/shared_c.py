import numpy as np


class SharedConstants:
    """Contains constants that are shared across both the supervisor and drones.

    All attributes defined below are the default values.
    """

    # An identifier
    name = "shared"

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
    use_exclusion = True

    def __init__(self, config: dict = {}, rng=None):
        """Initializes the algorithm's shared constants from a dictionary
        of key and value pairs.

        If a dictionary is not passed or a value is of a mistmatching type, the default values will be used.
        Additionally, a key will be ignored if it is not a valid parameter in this class.

        Args:
            config (dict, optional): A dictionary containing configuration parameters where keys
                match class attribute names and values are the desired settings. Defaults to {}.
            rng (numpy.random.Generator, optional): A pre-initialized NumPy random number generator,
                which ensures reproducability and stochasticty across different simulations. If None,
                a new generator is created using the instance's seed. Defaults to None.
        """
        # Set the values contained in a dictionary if provided
        for key, value in config.items():
            if value is not None:
                if not valid_parameter(self, key, value, self.name.upper()):
                    continue

                setattr(self, key, value)

        # If a numpy rng generator is not provided, initialize one
        if rng is None:
            self.rng = np.random.default_rng(self.seed)
        else:
            self.rng = rng


def valid_parameter(o: object, key: str, value, class_name: str) -> bool:
    """Ensures a `key` and `value` pair are valid attributes and assignments for `o`."""
    try:
        cls_value = getattr(o, key)
    except AttributeError:
        print(
            f"[{class_name}]: The following parameter '{key} is not supported, ignoring..."
        )
        return False

    if type(cls_value) is not type(value):
        print(
            f"""
            [{class_name}]: The provided value for {cls_value}: {value}, is of the incorrect type! 
            {type(cls_value)} != {type(value)}
            Using default value...
            """
        )
        return False
