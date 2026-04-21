from controller import Supervisor
import json
import math
import numpy as np
import os
import sys

current_dir = os.path.dirname(__file__)
shared_path = os.path.abspath(os.path.join(current_dir, "..", "shared"))
sys.path.append(shared_path)

import equations as eq
from supervisor_constants import *


def clamp(value, value_min, value_max):
    return max(value_min, min(value, value_max))


def world_to_grid(x, y, x_min, x_max, y_min, y_max, rows, cols):
    x_ratio = (x - x_min) / (x_max - x_min)
    y_ratio = (y - y_min) / (y_max - y_min)

    x_ratio = clamp(x_ratio, 0.0, 1.0)
    y_ratio = clamp(y_ratio, 0.0, 1.0)

    col = int(round(x_ratio * (cols - 1)))
    row = int(round(y_ratio * (rows - 1)))

    return row, col


def grid_to_world(row, col, x_min, x_max, y_min, y_max, rows, cols):
    x_ratio = col / max(cols - 1, 1)
    y_ratio = row / max(rows - 1, 1)

    x = x_min + x_ratio * (x_max - x_min)
    y = y_min + y_ratio * (y_max - y_min)
    return x, y

def build_border_decay_pheromone_map(rows, cols, p_max, lam):
    p = np.zeros((rows, cols), dtype=float)
    for i in range(rows):
        for j in range(cols):
            dist = min(i, rows - 1 - i, j, cols - 1 - j)
            p[i, j] = p_max * (lam ** dist)
    return p


def get_neighbor_cells(row, col, rows, cols):
    neighbors = []

    for d_row in (-1, 0, 1):
        for d_col in (-1, 0, 1):
            if d_row == 0 and d_col == 0:
                continue

            n_row = row + d_row
            n_col = col + d_col

            if 0 <= n_row < rows and 0 <= n_col < cols:
                neighbors.append((n_row, n_col))

    return neighbors


def add_pheromone_noise(new_map, p_max, noise_fraction):
    noisy = new_map.copy()

    if noise_fraction <= 0.0:
        return noisy

    interior = noisy[1:-1, 1:-1]
    noise_bound = noise_fraction * p_max
    noise = rng.uniform(-noise_bound, noise_bound, size=interior.shape)

    mask = (interior > 0.0) & (interior < p_max)
    interior[mask] = np.clip(interior[mask] + noise[mask], 0.0, p_max)

    noisy[1:-1, 1:-1] = interior
    return noisy

def update_pheromone_map_local(p_map, drone_grid_positions, p_max, alpha_pheromone, lam, update_radius, noise_fraction):
    rows, cols = p_map.shape
    new_map = p_map.copy()
    cells_to_update = set()

    num_drones = max(len(drone_grid_positions), 1)

    for drone_row, drone_col in drone_grid_positions:
        row_min = max(1, drone_row - update_radius)
        row_max = min(rows - 2, drone_row + update_radius)
        col_min = max(1, drone_col - update_radius)
        col_max = min(cols - 2, drone_col + update_radius)

        for row in range(row_min, row_max + 1):
            for col in range(col_min, col_max + 1):
                cells_to_update.add((row, col))

    for row, col in cells_to_update:
        total = 0.0
        for drone_row, drone_col in drone_grid_positions:
            total += lam ** max(abs(row - drone_row), abs(col - drone_col))

        # Average instead of raw sum to prevent immediate saturation
        p_new = p_max * (total / num_drones)

        value = alpha_pheromone * p_map[row, col] + (1.0 - alpha_pheromone) * p_new
        new_map[row, col] = clamp(value, 0.0, p_max)

    new_map = add_pheromone_noise(
        new_map=new_map,
        p_max=p_max,
        noise_fraction=noise_fraction
    )

    for drone_row, drone_col in drone_grid_positions:
        new_map[drone_row, drone_col] = p_max

    # Ghost border stays saturated and should never be attractive
    new_map[0, :] = p_max
    new_map[-1, :] = p_max
    new_map[:, 0] = p_max
    new_map[:, -1] = p_max

    return new_map


def compute_priority_map(p_map, drone_grid_positions, epsilon):
    rows, cols = p_map.shape
    total_cells = rows * cols

    raw = np.zeros_like(p_map, dtype=float)
    for row in range(rows):
        for col in range(cols):
            raw[row, col] = eq.get_raw_inverted_priority(p_map[row, col], epsilon)

    flat = raw.flatten()
    order = np.argsort(flat)
    ranks = np.empty_like(order, dtype=float)
    ranks[order] = np.arange(total_cells, dtype=float)

    priority_map = np.zeros_like(p_map, dtype=float)
    for idx, rank in enumerate(ranks):
        row = idx // cols
        col = idx % cols
        priority_map[row, col] = eq.normalize_priority(rank, total_cells)

    for row, col in drone_grid_positions:
        if 0 <= row < rows and 0 <= col < cols:
            priority_map[row, col] = 0.0

    return priority_map


def build_neighbor_priority_list(priority_map, row, col, x_min, x_max, y_min, y_max, rows, cols):
    neighbors = []

    for n_row, n_col in get_neighbor_cells(row, col, rows, cols):
        world_x, world_y = grid_to_world(
            row=n_row,
            col=n_col,
            x_min=x_min,
            x_max=x_max,
            y_min=y_min,
            y_max=y_max,
            rows=rows,
            cols=cols
        )
        q_k = float(priority_map[n_row, n_col])
        neighbors.append(((world_x, world_y), q_k))

    return neighbors


def clamp_vector_norm(v, max_norm):
    norm = np.linalg.norm(v)
    if norm == 0.0 or norm <= max_norm:
        return v
    return (v / norm) * max_norm


def mark_observed_cells(observed_mask, center_row, center_col, radius):
    rows, cols = observed_mask.shape
    r_sq = radius * radius

    for row in range(max(0, center_row - radius), min(rows - 1, center_row + radius) + 1):
        for col in range(max(0, center_col - radius), min(cols - 1, center_col + radius) + 1):
            d_row = row - center_row
            d_col = col - center_col
            if d_row * d_row + d_col * d_col <= r_sq:
                observed_mask[row, col] = True


def get_coverage_percent(observed_mask):
    return 100.0 * np.count_nonzero(observed_mask) / observed_mask.size

def initalize_drones(supervisor_robot, number_of_drones, spawn_radius=10.0, spawn_height=0.5):
    root_node = supervisor_robot.getRoot()
    children_field = root_node.getField("children")
    drone_defs = []
    drone_channels = {}
    translation_fields = {}

    for i in range(number_of_drones):
        drone_def = f"DRONE{i}"
        drone_channel = 10 + i
        if number_of_drones == 1:
            initial_x = 0.0
            initial_y = 0.0
        else:
            angle = 2.0 * math.pi * i / number_of_drones
            initial_x = spawn_radius * math.cos(angle)
            initial_y = spawn_radius * math.sin(angle)

        initial_z = spawn_height

        drone_string = f"""
        DEF {drone_def} IacaCrazyflie {{
          translation {initial_x} {initial_y} {initial_z}
          controller "iaca_drone"
          receiverChannel {drone_channel}
        }}
        """

        children_field.importMFNodeFromString(-1, drone_string)
        node = supervisor_robot.getFromDef(drone_def)

        if node is None:
            raise ValueError(f"Failed to create {drone_def}")

        translation_field = node.getField("translation")

        if translation_field is None:
            raise ValueError(f"Could not get translation field for {drone_def}")

        position = translation_field.getSFVec3f()
        print(
            f"{drone_def}: starting position = "
            f"{position[0]:.3f}, {position[1]:.3f}, {position[2]:.3f}"
        )

        drone_defs.append(drone_def)
        drone_channels[drone_def] = drone_channel
        translation_fields[drone_def] = translation_field

    return drone_defs, drone_channels, translation_fields

# get supervisor instance
robot = Supervisor()

# add the drones to the webots area and set their initial positions
drone_defs, drone_channels, translation_fields = initalize_drones(robot, NUMBER_OF_DRONES)

timestep = int(robot.getBasicTimeStep())
dt = timestep / 1000.0

emitter = robot.getDevice("cmd_emitter")

pheromone_map = build_border_decay_pheromone_map(GRID_ROWS, GRID_COLS, P_MAX, LAMBDA)
observed_mask = np.zeros((GRID_ROWS, GRID_COLS), dtype=bool)

# Numpy RNG
rng = np.random.default_rng(SEED)

paths = {drone_def: [] for drone_def in drone_defs}
coverage_history = []

step_count = 0

print("IACA supervisor started")
while robot.step(timestep) != -1:
    current_time = robot.getTime()
    step_count += 1

    drone_states = {}
    drone_grid_positions = []
    if step_count < MAX_STEPS and step_count % SUPERVISOR_STEP_SIZE != 0:
        continue

    for drone_def in drone_defs:
        pos = translation_fields[drone_def].getSFVec3f()
        x = pos[0]
        y = pos[1]
        z = pos[2]

        grid_row, grid_col = world_to_grid(
            x=x,
            y=y,
            x_min=WORLD_X_MIN,
            x_max=WORLD_X_MAX,
            y_min=WORLD_Y_MIN,
            y_max=WORLD_Y_MAX,
            rows=GRID_ROWS,
            cols=GRID_COLS
        )

        drone_states[drone_def] = {
            "x": x,
            "y": y,
            "z": z,
            "grid_row": grid_row,
            "grid_col": grid_col,
        }

        drone_grid_positions.append((grid_row, grid_col))
        paths[drone_def].append((x, y))

        mark_observed_cells(observed_mask, grid_row, grid_col, SENSOR_RADIUS_CELLS)

    pheromone_map = update_pheromone_map_local(
        p_map=pheromone_map,
        drone_grid_positions=drone_grid_positions,
        p_max=P_MAX,
        alpha_pheromone=ALPHA_PHEROMONE,
        lam=LAMBDA,
        update_radius=SENSOR_RADIUS_CELLS,
        noise_fraction=0.05
    )

    priority_map = compute_priority_map(
        p_map=pheromone_map,
        drone_grid_positions=drone_grid_positions,
        epsilon=EPSILON
    )

    for drone_def in drone_defs:
        state = drone_states[drone_def]
        row = state["grid_row"]
        col = state["grid_col"]

        if current_time < STARTUP_HOVER_TIME:
            # During startup just send hover with empty neighbors
            command = {
                "neighbors": {},
                "drone_row": row,
                "drone_col": col,
                "yaw_desired": 0.0,
                "height_desired": HEIGHT_DESIRED,
                "startup": True,
            }
        else:
            neighbor_cells = get_neighbor_cells(row, col, GRID_ROWS, GRID_COLS)
            neighbor_priorities = {}
            for n_row, n_col in neighbor_cells:
                # Convert grid position to world coords so drone can compute r_k
                world_x, world_y = grid_to_world(
                    row=n_row,
                    col=n_col,
                    x_min=WORLD_X_MIN,
                    x_max=WORLD_X_MAX,
                    y_min=WORLD_Y_MIN,
                    y_max=WORLD_Y_MAX,
                    rows=GRID_ROWS,
                    cols=GRID_COLS
                )
                key = f"{n_row},{n_col}"
                neighbor_priorities[key] = {
                    "q": float(priority_map[n_row, n_col]),
                    "wx": world_x,
                    "wy": world_y,
                }

            command = {
                "neighbors": neighbor_priorities,
                "drone_row": row,
                "drone_col": col,
                "yaw_desired": 0.0,
                "height_desired": HEIGHT_DESIRED,
                "startup": False,
            }

        emitter.setChannel(drone_channels[drone_def])
        payload = json.dumps(command).encode("utf-8")
        emitter.send(payload)

    coverage = get_coverage_percent(observed_mask)
    coverage_history.append(coverage)

    if step_count % PRINT_INTERVAL == 0:
        print(f"{current_time}: Step {step_count} coverage={coverage:.2f}%")

    if step_count >= MAX_STEPS:
        output_dir = os.path.dirname(__file__)
        temp_path = os.path.join(output_dir, "iaca_run_output.tmp.npz")
        output_path = os.path.join(output_dir, "iaca_run_output.npz")

        save_data = {
            "world_x_min": WORLD_X_MIN,
            "world_x_max": WORLD_X_MAX,
            "world_y_min": WORLD_Y_MIN,
            "world_y_max": WORLD_Y_MAX,
            "grid_rows": GRID_ROWS,
            "grid_cols": GRID_COLS,
            "coverage_history": np.array(coverage_history, dtype=np.float32),
        }

        for drone_def in drone_defs:
            save_data[f"{drone_def.lower()}_path"] = np.array(paths[drone_def], dtype=np.float32)

        np.savez_compressed(temp_path, **save_data)
        os.replace(temp_path, output_path)

        print(f"Saved run output to: {output_path}")
        robot.simulationSetMode(Supervisor.SIMULATION_MODE_PAUSE)
