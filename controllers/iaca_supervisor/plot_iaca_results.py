import numpy as np
import matplotlib.pyplot as plt

from supervisor_constants import *


def load_data(path):
    return np.load(path)


def plot_coverage(coverage_history):
    x = [i * SUPERVISOR_STEP_SIZE for i in range(len(coverage_history))]

    plt.figure()
    plt.plot(x, coverage_history)
    plt.xlabel("Step")
    plt.ylabel("Coverage (%)")
    plt.title("Coverage Over Time")
    plt.grid()
    plt.show()


def plot_paths(data):
    bounds = {
        "x_min": float(data["world_x_min"]),
        "x_max": float(data["world_x_max"]),
        "y_min": float(data["world_y_min"]),
        "y_max": float(data["world_y_max"]),
    }

    plt.figure()

    for i in range(NUMBER_OF_DRONES):
        key = f"drone{i}_path"
        path = data[key]

        if len(path) == 0:
            continue

        xs = path[:, 0]
        ys = path[:, 1]

        # Path
        plt.plot(xs, ys, label=f"DRONE{i}", alpha=0.7)

        # Start
        plt.plot(xs[0], ys[0], 'go', markersize=6)

        # End
        plt.plot(xs[-1], ys[-1], 'rx', markersize=8, markeredgewidth=2)

    plt.xlim(bounds["x_min"], bounds["x_max"])
    plt.ylim(bounds["y_min"], bounds["y_max"])

    plt.xlabel("X")
    plt.ylabel("Y")
    plt.title("Drone Paths (Start = green, End = red X)")
    plt.legend()
    plt.gca().set_aspect("equal", adjustable="box")
    plt.grid()
    plt.show()


def plot_heatmap(matrix, title):
    rows, cols = matrix.shape

    plt.figure()
    plt.imshow(matrix, origin="lower")
    plt.colorbar()
    plt.xticks([0, cols - 1])
    plt.yticks([0, rows - 1])
    plt.title(title)
    plt.show()


def plot_maps(data, snapshot_index=-1):
    """
    Plot pheromone + priority maps at a given snapshot index.
    Default = last snapshot.
    """
    pheromone_snapshots = data["pheromone_snapshots"]
    priority_snapshots = data["priority_snapshots"]

    if len(pheromone_snapshots) == 0:
        print("No snapshots saved.")
        return

    # Handle negative indexing (default = last)
    snapshot_index = snapshot_index % len(pheromone_snapshots)

    pheromone_map = pheromone_snapshots[snapshot_index]
    priority_map = priority_snapshots[snapshot_index]

    print(f"Plotting snapshot {snapshot_index} / {len(pheromone_snapshots) - 1}")

    plot_heatmap(pheromone_map, f"Pheromone Map (snapshot {snapshot_index} / {len(pheromone_snapshots) - 1})")
    plot_heatmap(priority_map, f"Priority Map (snapshot {snapshot_index} / {len(priority_snapshots) - 1})")


def plot_everything(npz_path, snapshot_index=-1):
    data = load_data(npz_path)

    coverage_history = data["coverage_history"]

    print(f"Final coverage: {coverage_history[-1]:.2f}%")

    plot_coverage(coverage_history)
    plot_paths(data)
    plot_maps(data, snapshot_index)


if __name__ == "__main__":
    plot_everything("iaca_run_output.npz", snapshot_index=-1)
