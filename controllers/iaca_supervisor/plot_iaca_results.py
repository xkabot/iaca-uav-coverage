import numpy as np
import matplotlib.pyplot as plt

from supervisor_constants import *


def load_data(path):
    """
    Load the compressed NumPy run output.
    :param path: path to .npz file
    :return: loaded npz object
    """
    return np.load(path)


def plot_coverage(coverage_history):
    """
    Plot coverage percentage over time.
    :param coverage_history: 1D array of coverage values
    :return: None
    """
    x = [i * SUPERVISOR_STEP_SIZE for i in range(len(coverage_history))]

    plt.figure()
    plt.plot(x, coverage_history)
    plt.xlabel("Step")
    plt.ylabel("Coverage (%)")
    plt.title("Coverage Over Time")
    plt.grid()
    plt.show()


def plot_paths(data):
    """
    Plot drone paths in world coordinates.
    :param data: loaded npz object
    :return: None
    """
    bounds = {
        "x_min": float(data["world_x_min"]),
        "x_max": float(data["world_x_max"]),
        "y_min": float(data["world_y_min"]),
        "y_max": float(data["world_y_max"]),
    }

    path_names = [
        "drone0_path",
        "drone1_path",
        "drone2_path",
        "drone3_path",
    ]

    plt.figure()

    for idx, key in enumerate(path_names):
        path = data[key]

        if len(path) == 0:
            continue

        xs = path[:, 0]
        ys = path[:, 1]

        plt.plot(xs, ys, label=f"DRONE{idx}")

    plt.xlim(bounds["x_min"], bounds["x_max"])
    plt.ylim(bounds["y_min"], bounds["y_max"])

    plt.xlabel("X")
    plt.ylabel("Y")
    plt.title("Drone Paths")
    plt.legend()
    plt.gca().set_aspect("equal", adjustable="box")
    plt.grid()
    plt.show()


def plot_everything(npz_path):
    """
    Load and plot all saved data from the compressed run output.
    :param npz_path: path to .npz file
    :return: None
    """
    data = load_data(npz_path)

    coverage_history = data["coverage_history"]

    print(f"Final coverage: {coverage_history[-1]:.2f}%")

    plot_coverage(coverage_history)
    plot_paths(data)


if __name__ == "__main__":
    plot_everything("iaca_run_output.npz")