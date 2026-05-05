import argparse
import os, sys
import numpy as np
import matplotlib.image as mpimg
import matplotlib.pyplot as plt

current_dir = os.path.dirname(__file__)
supervisor_path = os.path.abspath(os.path.join(current_dir, "..", "controllers", "iaca_supervisor"))
pics_path = os.path.abspath(os.path.join(current_dir, "..", "pics"))
sys.path.append(supervisor_path)
sys.path.append(pics_path)

from supervisor_c import make_exclusion_mask

BACKGROUND_IMG_PATH = os.path.join(pics_path, "background.png")
BACKGROUND_IMG = mpimg.imread(BACKGROUND_IMG_PATH)

BUFFER = 100.0


def load_data(path) -> dict:
    data = np.load(path)
    data_dict = dict(data)
    data.close()
    return data_dict


def draw_background(ax, img, bounds):
    ax.imshow(
        img,
        extent=[
            bounds["x_min"],
            bounds["x_max"],
            bounds["y_min"],
            bounds["y_max"],
        ],
        origin="upper"
    )


def expand_bounds(bounds, buffer):
    return {
        "x_min": bounds["x_min"] - buffer,
        "x_max": bounds["x_max"] + buffer,
        "y_min": bounds["y_min"] - buffer,
        "y_max": bounds["y_max"] + buffer,
    }


def plot_coverage(coverage_history, supervisor_step):
    x = [i * supervisor_step for i in range(len(coverage_history))]

    plt.figure()
    plt.plot(x, coverage_history)
    plt.xlabel("Step")
    plt.ylabel("Coverage (%)")
    plt.title("Coverage Over Time")
    plt.grid()
    plt.show()


def plot_paths(data, exclusion_mask=None):
    base_bounds = {
        "x_min": float(data["world_x_min"]),
        "x_max": float(data["world_x_max"]),
        "y_min": float(data["world_y_min"]),
        "y_max": float(data["world_y_max"]),
    }

    bounds = expand_bounds(base_bounds, BUFFER)

    fig, ax = plt.subplots()
    ax.set_facecolor("white")
    draw_background(ax, BACKGROUND_IMG, base_bounds)

    # Draw exclusion zones as a partially transparent red overlay
    if exclusion_mask is not None:
        overlay = np.zeros((*exclusion_mask.shape, 4), dtype=float)
        overlay[exclusion_mask] = [1.0, 0.0, 0.0, 0.4]  # red, 40% opacity
        ax.imshow(
            overlay,
            origin="lower",
            extent=[
                base_bounds["x_min"],
                base_bounds["x_max"],
                base_bounds["y_min"],
                base_bounds["y_max"],
            ],
            interpolation="nearest"
        )

    for i in range(data["number_of_drones"]):
        key = f"drone{i}_path"
        path = data[key]

        if len(path) == 0:
            continue

        xs = path[:, 0]
        ys = path[:, 1]

        ax.plot(xs, ys, label=f"DRONE{i}", alpha=0.7)
        ax.plot(xs[0], ys[0], 'go', markersize=6)
        ax.plot(xs[-1], ys[-1], 'rx', markersize=8, markeredgewidth=2)

    ax.set_xlim(bounds["x_min"], bounds["x_max"])
    ax.set_ylim(bounds["y_min"], bounds["y_max"])
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_title("Drone Paths (Start = green, End = red X)")
    ax.legend()
    ax.set_aspect("equal", adjustable="box")
    ax.grid()
    plt.show()


def plot_heatmap(matrix, title, base_bounds):
    fig, ax = plt.subplots()
    ax.set_facecolor("white")
    # Background
    draw_background(ax, BACKGROUND_IMG, base_bounds)

    im = ax.imshow(
        matrix,
        origin="lower",
        extent=[
            base_bounds["x_min"],
            base_bounds["x_max"],
            base_bounds["y_min"],
            base_bounds["y_max"],
        ],
        vmin=np.min(matrix),
        vmax=np.max(matrix),
        alpha=0.75
    )

    plt.colorbar(im, ax=ax)

    ax.set_xlim(base_bounds["x_min"], base_bounds["x_max"])
    ax.set_ylim(base_bounds["y_min"], base_bounds["y_max"])

    ax.set_title(title)
    ax.set_xlabel("X")
    ax.set_ylabel("Y")

    plt.show()


def plot_maps(data, snapshot_index=-1):
    pheromone_snapshots = data["pheromone_snapshots"]
    priority_snapshots = data["priority_snapshots"]

    if len(pheromone_snapshots) == 0:
        print("No snapshots saved.")
        return

    bounds = {
        "x_min": float(data["world_x_min"]),
        "x_max": float(data["world_x_max"]),
        "y_min": float(data["world_y_min"]),
        "y_max": float(data["world_y_max"]),
    }

    snapshot_index = snapshot_index % len(pheromone_snapshots)

    pheromone_map = pheromone_snapshots[snapshot_index]
    priority_map = priority_snapshots[snapshot_index]

    print(f"Plotting snapshot {snapshot_index} / {len(pheromone_snapshots) - 1}")

    plot_heatmap(
        pheromone_map,
        f"Pheromone Map (snapshot {snapshot_index} / {len(pheromone_snapshots) - 1})",
        bounds
    )

    plot_heatmap(
        priority_map,
        f"Priority Map (snapshot {snapshot_index} / {len(pheromone_snapshots) - 1})",
        bounds
    )


def plot_everything(npz_path, snapshot_index=-1):
    data = load_data(npz_path)

    coverage_history = data["coverage_history"]
    supervisor_step = data["supervisor_step"]
    use_exclusion = data["use_exclusion"]

    print(f"Final coverage: {coverage_history[-1]:.2f}%")

    if use_exclusion:
        exclusion_mask = make_exclusion_mask(data)
    else:
        exclusion_mask = None

    plot_coverage(coverage_history, supervisor_step)
    plot_paths(data, exclusion_mask=exclusion_mask)
    plot_maps(data, snapshot_index)


if __name__ == "__main__":
    # Define the configuration to animate
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=str, default="single_run")
    args = parser.parse_args()
    
    config_dir = args.config
    
    plot_everything(os.path.join(supervisor_path, "out", config_dir, "iaca_run_output.npz"), snapshot_index=-1)
