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

    plt.figure()

    # plot main paths
    for i in range(NUMBER_OF_DRONES):
        key = f"drone{i}_path"
        path = data[key]

        if len(path) == 0:
            continue

        xs = path[:, 0]
        ys = path[:, 1]

        plt.plot(xs, ys, label=f"DRONE{i}")

        # add an x to the end of the path to show where the drone ended up
        end_x = xs[-1]
        end_y = ys[-1]
        plt.plot(end_x, end_y, 'rx', markersize=7, markeredgewidth=2)

    # plot start points
    for i in range(NUMBER_OF_DRONES):
        key = f"drone{i}_path"
        path = data[key]

        xs = path[:, 0]
        ys = path[:, 1]

        start_x = xs[0]
        start_y = ys[0]
        # add a circle at the start of the path to show where the drone started
        plt.plot(start_x, start_y, 'go', markersize=7, markeredgewidth=2)


    # plot end points
    for i in range(NUMBER_OF_DRONES):
        key = f"drone{i}_path"
        path = data[key]

        xs = path[:, 0]
        ys = path[:, 1]

        end_x = xs[-1]
        end_y = ys[-1]
        # add an x to the end of the path to show where the drone ended up
        plt.plot(end_x, end_y, 'rx', markersize=7, markeredgewidth=2)


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