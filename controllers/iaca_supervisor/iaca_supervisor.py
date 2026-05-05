import pickle
import subprocess

from controller import Supervisor
from scipy.ndimage import distance_transform_edt
import json
import math
import numpy as np
import os
import sys

current_dir = os.path.dirname(__file__)
shared_path = os.path.abspath(os.path.join(current_dir, "..", "shared"))
config_path = os.path.abspath(os.path.join(current_dir, "..", "config"))
sys.path.append(shared_path)
sys.path.append(config_path)

import equations as eq
from supervisor_c import SupervisorConstants
from shared_c import SharedConstants


def load_config() -> dict:
    """Loads a config from a json file."""
    
    cfg_file = os.path.join(config_path, "configs.json")
    print(f"Supervisor reading: {cfg_file}")
    with open(cfg_file, "r") as file:
        return json.load(file)
    
def init_configs(cfg: dict, rng, experimenting=False) -> SupervisorConstants:
    """Initializes a `SupervisorConstants` instance, which contains the values provided
    by `cfg`. Any values not present in `cfg` will contain their default values defined
    in `SupervisorConstants` and `SharedConstants`."""
    
    if experimenting:
        shared = SharedConstants(cfg["shared"], rng)
        supervisor = SupervisorConstants(shared, cfg["supervisor"])
    else:
        # If not experimenting, just initialize with default values.
        shared = SharedConstants(rng=rng)
        supervisor = SupervisorConstants(shared)
        
    return supervisor


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
    rel_noise_bound = noise_fraction * interior
    noise = cfg.rng.uniform(-1.0, 1.0, size=interior.shape) * rel_noise_bound

    mask = (interior > 0.0) & (interior < p_max)
    interior[mask] = np.clip(interior[mask] + noise[mask], 0.0, p_max)

    noisy[1:-1, 1:-1] = interior
    return noisy


def update_pheromone_map(p_map, drone_grid_positions, p_max, alpha_pheromone, lam, noise_fraction, exclusion_mask=None):
    rows, cols = p_map.shape
    drone_positions = np.array(drone_grid_positions)  # shape (N, 2)

    # Build 2D grid of indices, shape (rows, cols)
    R, C = np.meshgrid(np.arange(rows), np.arange(cols), indexing='ij')

    # Chebyshev distance from every cell to every drone, shape (N, rows, cols)
    dr = np.abs(R[np.newaxis, :, :] - drone_positions[:, 0, np.newaxis, np.newaxis])
    dc = np.abs(C[np.newaxis, :, :] - drone_positions[:, 1, np.newaxis, np.newaxis])
    chebyshev = np.maximum(dr, dc)

    # Sum contributions across drones, shape (rows, cols)
    p_new = p_max * np.sum(lam ** chebyshev, axis=0)
    p_new = np.clip(p_new, 0.0, p_max)

    # Noise on p_new before merging
    p_new = add_pheromone_noise(new_map=p_new, p_max=p_max, noise_fraction=noise_fraction)

    # exponential smoothing over full map
    new_map = eq.get_updated_pheromone_cell(p_map, p_new, alpha_pheromone)
    new_map = np.clip(new_map, 0.0, p_max)

    # Max out excluded cells - max pheromones so drones avoid them
    if exclusion_mask is not None:
        new_map[exclusion_mask] = p_max

    # Saturate drone cells and borders
    new_map[tuple(drone_positions.T)] = p_max
    new_map[0, :] = p_max
    new_map[-1, :] = p_max
    new_map[:, 0] = p_max
    new_map[:, -1] = p_max

    return new_map


def compute_priority_map(p_map, drone_grid_positions, epsilon, exclusion_mask=None):
    rows, cols = p_map.shape
    total_cells = rows * cols

    # Eq 4: raw inversion
    raw = eq.get_raw_inverted_priority(p_map, epsilon)

    # Eq 5: rank-based normalization
    flat = raw.flatten()
    ranks = np.empty_like(flat, dtype=float)
    ranks[np.argsort(flat)] = np.arange(total_cells, dtype=float)
    priority_map = eq.normalize_priority(ranks, rows, cols)

    # Zero out excluded cells - min priority so drones avoid them
    if exclusion_mask is not None:
        priority_map[exclusion_mask] = 0.0

    # Zero out drone positions
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


def get_coverage_percent(observed_mask, exclusion_mask=None):
    if not exclusion_mask:
        return 100.0 * np.count_nonzero(observed_mask) / observed_mask.size

    valid_mask = ~exclusion_mask

    valid_cell_count = np.count_nonzero(valid_mask)
    if valid_cell_count == 0:
        return 0.0

    covered_valid_mask = observed_mask & valid_mask

    return 100.0 * np.count_nonzero(covered_valid_mask) / valid_cell_count


def initalize_drones(supervisor_robot, number_of_drones, spawn_radius, spawn_height=0.5):
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
            yaw_angle = 0.0
        else:
            angle = 2.0 * math.pi * i / number_of_drones
            initial_x = spawn_radius * math.cos(angle)
            initial_y = spawn_radius * math.sin(angle)
            yaw_angle = angle

        initial_z = spawn_height

        sanitized_rng_path = rng_state_file.replace("\\", "/")
        config_str = f"{sanitized_rng_path},{config_index}"

        drone_string = f"""
        DEF {drone_def} IacaCrazyflie {{
          translation {initial_x} {initial_y} {initial_z}
          rotation 0 0 1 {yaw_angle}
          controller "iaca_drone"
          receiverChannel {drone_channel}
          customData "{config_str}"
        }}
        """

        children_field.importMFNodeFromString(-1, drone_string)
        
        # Immediately grab the last node added to the children field
        # This is much faster and more reliable than searching by DEF name
        node = children_field.getMFNode(-1)
        
        if node.getDef() != drone_def:
            node = supervisor_robot.getFromDef(drone_def)

        if node is None:
            raise ValueError(f"Failed to create {drone_def}")

        translation_field = node.getField("translation")

        if translation_field is None:
            raise ValueError(f"Could not get translation field for {drone_def}")

        position = translation_field.getSFVec3f()
        # print(
        #     f"{drone_def}: starting position = "
        #     f"{position[0]:.3f}, {position[1]:.3f}, {position[2]:.3f}"
        # )

        drone_defs.append(drone_def)
        drone_channels[drone_def] = drone_channel
        translation_fields[drone_def] = translation_field

    return drone_defs, drone_channels, translation_fields


def build_exclusion_gradient(exclusion_mask, rows, cols, margin_cells=10):
    safe_mask = ~exclusion_mask

    # Distance of each safe cell to nearest excluded cell (for margin effect)
    dist_to_exclusion, indices_to_nearest_safe = distance_transform_edt(
        exclusion_mask, return_indices=True
    )

    # Also get distance of safe cells to nearest excluded cell
    dist_safe_to_exclusion = distance_transform_edt(safe_mask)

    R, C = np.meshgrid(np.arange(rows), np.arange(cols), indexing='ij')
    grad_row = indices_to_nearest_safe[0] - R
    grad_col = indices_to_nearest_safe[1] - C

    mag = np.sqrt(grad_row ** 2 + grad_col ** 2)
    mag = np.where(mag < 1e-6, 1.0, mag)
    grad_row = grad_row / mag
    grad_col = grad_col / mag

    # In safe cells near the boundary, point away from exclusion zone
    near_boundary = safe_mask & (dist_safe_to_exclusion < margin_cells)
    # For these cells, flip to point away from exclusion
    grad_row[safe_mask & ~near_boundary] = 0.0
    grad_col[safe_mask & ~near_boundary] = 0.0

    # Scale by proximity — full strength at boundary, zero at margin edge
    strength_scale = np.where(
        near_boundary,
        1.0 - dist_safe_to_exclusion / margin_cells,
        0.0
    )
    grad_row = grad_row * strength_scale
    grad_col = grad_col * strength_scale

    # Excluded cells always get full escape direction
    grad_row[exclusion_mask] = (indices_to_nearest_safe[0][exclusion_mask] - R[exclusion_mask]) / mag[exclusion_mask]
    grad_col[exclusion_mask] = (indices_to_nearest_safe[1][exclusion_mask] - C[exclusion_mask]) / mag[exclusion_mask]

    return grad_row, grad_col


# get supervisor instance
robot = Supervisor()

# Retrieve the entire config (contains run configs and constants)
master_config = load_config()
configs = master_config.get("configs", [{}])
experimenting = master_config.get("experimenting", False)

# Define the random number generator
rng = np.random.default_rng(master_config["seed"])

# Run once if experiments are not being simulated
if experimenting:
    num_sims = master_config["num_sims"]
    num_configs = len(master_config["configs"])
else:
    num_sims, num_configs = 1, 1

# Output file directories
output_dir = master_config["output_dir"]
temp_dir = master_config["temp_dir"]

# Output file paths/names
final_results_file = os.path.join(current_dir, output_dir, master_config["final_coverage"])
# Defines the output for the first simulation results
iaca_run_path = os.path.join(current_dir, output_dir, master_config["iaca_run"])
rng_state_file = os.path.join(shared_path, temp_dir, master_config["rng_state_file"])

# Store coverage data
coverages = {}


# Test each configuration to simulate, execute a standard run if experimenting is False
for config_index in range(num_configs):
    if experimenting:
        cfg = init_configs(master_config["configs"][config_index], rng, experimenting)
        cfg_name = master_config["configs"][config_index].get("name", f"config{config_index}")
    else:
        cfg = init_configs({}, rng)
        cfg_name = "single_run"
        
    print(f"Now testing {cfg_name}...")
        
    # Store the total coverage across all simulations
    coverage_sum = 0
    for sim in range(num_sims):    
        # Save the state of the rng generator, ensures stochastic results between sims
        with open(rng_state_file, "wb") as f:
            pickle.dump(rng, f)
        
        # add the drones to the webots area and set their initial positions
        drone_defs, drone_channels, translation_fields = initalize_drones(robot, cfg.number_of_drones, cfg.spawn_radius)

        timestep = int(robot.getBasicTimeStep())
        dt = timestep / 1000.0

        emitter = robot.getDevice("cmd_emitter")

        pheromone_map = build_border_decay_pheromone_map(cfg.grid_rows, cfg.grid_cols, cfg.p_max, cfg.lam)
        observed_mask = np.zeros((cfg.grid_rows, cfg.grid_cols), dtype=bool)
        priority_map = np.zeros((cfg.grid_rows, cfg.grid_cols), dtype=float)

        if cfg.use_exclusion:
            pheromone_map[cfg.exclusion_mask] = cfg.p_max
            exclusion_grad_row, exclusion_grad_col = build_exclusion_gradient(
                cfg.exclusion_mask, 
                cfg.grid_rows, 
                cfg.grid_cols, 
                margin_cells=cfg.exclusion_margin_cells
            )
        else:
            exclusion_grad_row, exclusion_grad_col = None, None

        paths = {drone_def: [] for drone_def in drone_defs}
        coverage_history = []
        pheromone_snapshots = []
        priority_snapshots = []
        snapshot_steps = []
        snapshot_times = []

        step_count = 0

        print("IACA supervisor started")
        while robot.step(timestep) != -1:
            current_time = robot.getTime()
            step_count += 1

            is_supervisor_step = (step_count % cfg.supervisor_step_size == 0)
            is_save_step = (step_count % cfg.save_maps_interval == 0)
            is_final_step = (step_count >= cfg.max_steps)

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
                    x_min=cfg.world_x_min,
                    x_max=cfg.world_x_max,
                    y_min=cfg.world_y_min,
                    y_max=cfg.world_y_max,
                    rows=cfg.grid_rows,
                    cols=cfg.grid_cols
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

                mark_observed_cells(observed_mask, grid_row, grid_col, cfg.sensor_radius_cells)

            # Pheromone/priority only update on supervisor cadence, never because of a save
            if is_supervisor_step or is_final_step:
                pheromone_map = update_pheromone_map(
                    p_map=pheromone_map,
                    drone_grid_positions=drone_grid_positions,
                    p_max=cfg.p_max,
                    alpha_pheromone=cfg.alpha_pheromone,
                    lam=cfg.lam,
                    noise_fraction=0.05,
                    exclusion_mask=cfg.exclusion_mask
                )
                priority_map = compute_priority_map(
                    p_map=pheromone_map,
                    drone_grid_positions=drone_grid_positions,
                    epsilon=cfg.epsilon,
                    exclusion_mask=cfg.exclusion_mask
                )

            if is_save_step or is_final_step:
                pheromone_snapshots.append(pheromone_map.copy().astype(np.float32))
                priority_snapshots.append(priority_map.copy().astype(np.float32))
                snapshot_steps.append(step_count)
                snapshot_times.append(current_time)

            # Command sending also independent
            if is_supervisor_step or is_final_step:
                for drone_def in drone_defs:
                    state = drone_states[drone_def]
                    row = state["grid_row"]
                    col = state["grid_col"]

                    if current_time < cfg.startup_hover_time:
                        # During startup just send hover with empty neighbors
                        command = {
                            "neighbors": {},
                            "drone_row": row,
                            "drone_col": col,
                            "yaw_desired": 0.0,
                            "height_desired": cfg.height_desired,
                            "startup": True,
                        }
                    else:
                        neighbor_cells = get_neighbor_cells(row, col, cfg.grid_rows, cfg.grid_cols)
                        neighbor_priorities = {}
                        for n_row, n_col in neighbor_cells:
                            # Convert grid position to world coords so drone can compute r_k
                            world_x, world_y = grid_to_world(
                                row=n_row,
                                col=n_col,
                                x_min=cfg.world_x_min,
                                x_max=cfg.world_x_max,
                                y_min=cfg.world_y_min,
                                y_max=cfg.world_y_max,
                                rows=cfg.grid_rows,
                                cols=cfg.grid_cols
                            )
                            key = f"{n_row},{n_col}"
                            neighbor_priorities[key] = {
                                "q": float(priority_map[n_row, n_col]),
                                "wx": world_x,
                                "wy": world_y,
                                "excluded": bool(cfg.exclusion_mask[n_row, n_col]) if cfg.exclusion_mask else False
                            }
                            
                        if not cfg.exclusion_mask:
                            escape_r = float(exclusion_grad_row[row, col])
                            escape_c = float(exclusion_grad_col[row, col])
                        else:
                            escape_r, escape_c = 0.0, 0.0

                        command = {
                            "neighbors": neighbor_priorities,
                            "drone_row": row,
                            "drone_col": col,
                            "yaw_desired": 0.0,
                            "height_desired": cfg.height_desired,
                            "startup": False,
                            "escape_row": escape_r,
                            "escape_col": escape_c,
                        }

                    emitter.setChannel(drone_channels[drone_def])
                    payload = json.dumps(command).encode("utf-8")
                    emitter.send(payload)

                coverage = get_coverage_percent(observed_mask, cfg.exclusion_mask)
                coverage_history.append(coverage)

                if step_count % cfg.print_interval == 0:
                    #print(f"{current_time}: Step {step_count} coverage={coverage:.2f}%")
                    pass
                    
            if is_final_step:
                # Only save key results of the first simulation
                if sim == 0:
                    tmp_path = iaca_run_path + ".tmp.npz"
                    out_path = iaca_run_path + ".npz"

                    save_data = {
                        "world_x_min": cfg.world_x_min,
                        "world_x_max": cfg.world_x_max,
                        "world_y_min": cfg.world_y_min,
                        "world_y_max": cfg.world_y_max,
                        "grid_rows": cfg.grid_rows,
                        "grid_cols": cfg.grid_cols,
                        "coverage_history": np.array(coverage_history, dtype=np.float32),
                        "snapshot_steps": np.array(snapshot_steps, dtype=np.int32),
                        "snapshot_times": np.array(snapshot_times, dtype=np.float32),
                        "pheromone_snapshots": np.stack(pheromone_snapshots).astype(np.float32) if len(
                            pheromone_snapshots) > 0 else np.empty((0, cfg.grid_rows, cfg.grid_cols), dtype=np.float32),
                        "priority_snapshots": np.stack(priority_snapshots).astype(np.float32) if len(
                            priority_snapshots) > 0 else np.empty((0, cfg.grid_rows, cfg.grid_cols), dtype=np.float32),
                    }

                    for drone_def in drone_defs:
                        save_data[f"{drone_def.lower()}_path"] = np.array(paths[drone_def], dtype=np.float32)

                    np.savez_compressed(tmp_path, **save_data)
                    os.replace(tmp_path, out_path)
                    print(f"Saved run output to: {iaca_run_path}")
                
                # Move onto next simulation, or end program
                if step_count >= cfg.max_steps:
                    print(f"Sim {sim} finished. Final coverage: {coverage:.2f}%")
                    break
        
        # Add the final coverage value to sum for config simulation
        coverage_sum += coverage_history[-1]

        # If experiments are being ran, remove the drones and reset simulation
        if experimenting:
            for drone_def in drone_defs:
                node = robot.getFromDef(drone_def)
                if node:
                    node.remove()

            robot.simulationReset()
            
            if robot.step(timestep) == -1:
                print("Webots closed. Exiting.")
                sys.exit()
                
    # After all simulations are complete, average the coverage
    coverages[cfg_name] = round(coverage_sum / num_sims, 3)
    
    # Write the final results for each config
    with open(final_results_file, "w") as out:
        json.dump(coverages, out)


# Pause and save the coverage results
print("\nFinished all simulations!")

# Pause to prevent further simulation
robot.simulationSetMode(Supervisor.SIMULATION_MODE_PAUSE)

# Remove RNG state file
if os.path.exists(rng_state_file):
    os.remove(rng_state_file)
