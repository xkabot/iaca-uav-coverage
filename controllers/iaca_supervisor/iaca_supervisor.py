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
    noise = np.random.uniform(-noise_bound, noise_bound, size=interior.shape)

    mask = (interior > 0.0) & (interior < p_max)
    interior[mask] = np.clip(interior[mask] + noise[mask], 0.0, p_max)

    noisy[1:-1, 1:-1] = interior
    return noisy

def update_pheromone_map_local(p_map, drone_grid_positions, p_max, alpha_pheromone, lam, update_radius, noise_fraction):
    rows, cols = p_map.shape
    new_map = p_map.copy()

    num_drones = max(len(drone_grid_positions), 1)

    update_cells = np.zeros(p_map.shape)
    for drone_row, drone_col in drone_grid_positions:
        row_min = max(1, drone_row - update_radius)
        row_max = min(rows - 2, drone_row + update_radius)
        col_min = max(1, drone_col - update_radius)
        col_max = min(cols - 2, drone_col + update_radius)

        update_cells[row_min:row_max + 1, col_min:col_max +  1] = 1

    update_indices = np.argwhere(update_cells)  # Shape: (N, 2)
    drone_positions = np.array(drone_grid_positions)  # Shape: (M, 2)

    # Find the row and column distances between each pheromone cell and drone position
    # Resulting shape: (N, 1, 2) - (1, M, 2) = (N, M, 2)
    diff = np.abs(update_indices[:, np.newaxis, :] - drone_positions[np.newaxis, :, :])

    # Calculate the higher values between the row and column distances
    # Resulting shape: (N, M)
    max_diff = np.max(diff, axis=2)

    # Calculate the new unnormalized pheromone contributions
    # Resulting shape: (N,)
    total = np.sum(lam ** max_diff, axis=1)

    # Calculate the new pheromone value for each cell to update in the new pheromone matrix
    p_new = p_max * (total / num_drones)
    values = alpha_pheromone * p_map[tuple(update_indices.T)] + (1.0 - alpha_pheromone) * p_new
    new_map[tuple(update_indices.T)] = np.maximum(0.0, np.minimum(values, p_max))

    new_map = add_pheromone_noise(
        new_map=new_map,
        p_max=p_max,
        noise_fraction=noise_fraction
    )

    new_map[tuple(drone_positions.T)] = p_max

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


robot = Supervisor()
timestep = int(robot.getBasicTimeStep())
dt = timestep / 1000.0

emitter = robot.getDevice("cmd_emitter")

drone_defs = ["DRONE0", "DRONE1", "DRONE2", "DRONE3"]
drone_channels = {
    "DRONE0": 10,
    "DRONE1": 11,
    "DRONE2": 12,
    "DRONE3": 13,
}

translation_fields = {}

for drone_def in drone_defs:
    node = robot.getFromDef(drone_def)
    if node is None:
        raise ValueError(f"Could not find node with DEF name {drone_def}")
    translation_fields[drone_def] = node.getField("translation")

WORLD_X_MIN = -30.0
WORLD_X_MAX = 30.0
WORLD_Y_MIN = -30.0
WORLD_Y_MAX = 35.0

GRID_ROWS = 100
GRID_COLS = 100

P_MAX = 220.0
ALPHA_PHEROMONE = 0.99
LAMBDA = 0.9
EPSILON = 1e-30

HEIGHT_DESIRED = 2.0

STARTUP_HOVER_TIME = 3.0

SENSOR_RADIUS_CELLS = 3
MAX_STEPS = 15000
SAVE_INTERVAL = 25

pheromone_map = build_border_decay_pheromone_map(GRID_ROWS, GRID_COLS, P_MAX, LAMBDA)
observed_mask = np.zeros((GRID_ROWS, GRID_COLS), dtype=bool)


paths = {drone_def: [] for drone_def in drone_defs}
grid_paths = {drone_def: [] for drone_def in drone_defs}
coverage_history = []
pheromone_snapshots = []
priority_snapshots = []

step_count = 0

print("Force-based IACA supervisor with logging started")

while robot.step(timestep) != -1:
    current_time = robot.getTime()
    step_count += 1

    drone_states = {}
    drone_grid_positions = []

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
        grid_paths[drone_def].append((grid_row, grid_col))
        mark_observed_cells(observed_mask, grid_row, grid_col, SENSOR_RADIUS_CELLS)

    pheromone_map = update_pheromone_map_local(
        p_map=pheromone_map,
        drone_grid_positions=drone_grid_positions,
        p_max=P_MAX,
        alpha_pheromone=ALPHA_PHEROMONE,
        lam=LAMBDA,
        update_radius=20,
        noise_fraction=0.05
    )

    priority_map = compute_priority_map(
        p_map=pheromone_map,
        drone_grid_positions=drone_grid_positions,
        epsilon=EPSILON
    )

    if step_count % SAVE_INTERVAL == 0:
        pheromone_snapshots.append(pheromone_map.copy().tolist())
        priority_snapshots.append(priority_map.copy().tolist())

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

    if step_count % 25 == 0:
        print(f"Step {step_count} coverage={coverage:.2f}%")

    if step_count >= MAX_STEPS:
        output = {
            "world_bounds": {
                "x_min": WORLD_X_MIN,
                "x_max": WORLD_X_MAX,
                "y_min": WORLD_Y_MIN,
                "y_max": WORLD_Y_MAX,
            },
            "grid": {
                "rows": GRID_ROWS,
                "cols": GRID_COLS,
            },
            "params": {
                "p_max": P_MAX,
                "alpha_pheromone": ALPHA_PHEROMONE,
                "lambda": LAMBDA,
                "epsilon": EPSILON,
                "height_desired": HEIGHT_DESIRED,
                "startup_hover_time": STARTUP_HOVER_TIME,
                "sensor_radius_cells": SENSOR_RADIUS_CELLS,
                "max_steps": MAX_STEPS,
                "save_interval": SAVE_INTERVAL,
            },
            "coverage_history": coverage_history,
            "paths": paths,
            "grid_paths": grid_paths,
            "final_pheromone_map": pheromone_map.tolist(),
            "final_priority_map": priority_map.tolist(),
            "observed_mask": observed_mask.astype(int).tolist(),
            "pheromone_snapshots": pheromone_snapshots,
            "priority_snapshots": priority_snapshots,
        }

        output_path = os.path.join(os.path.dirname(__file__), "iaca_run_output.json")
        with open(output_path, "w", encoding="utf-8") as f:
            json.dump(output, f)

        print(f"Saved run output to: {output_path}")
        robot.simulationSetMode(Supervisor.SIMULATION_MODE_PAUSE)
