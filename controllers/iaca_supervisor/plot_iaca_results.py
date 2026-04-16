import json
import numpy as np
import matplotlib.pyplot as plt

from supervisor_constants import *

def load_data(path):
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


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
    paths = data["paths"]
    bounds = data["world_bounds"]

    plt.figure()

    for drone_name, path in paths.items():
        xs = [p[0] for p in path]
        ys = [p[1] for p in path]
        plt.plot(xs, ys, label=drone_name)

    plt.xlim(bounds["x_min"], bounds["x_max"])
    plt.ylim(bounds["y_min"], bounds["y_max"])

    plt.xlabel("X")
    plt.ylabel("Y")
    plt.title("Drone Paths")
    plt.legend()
    plt.gca().set_aspect("equal", adjustable="box")
    plt.grid()
    plt.show()


def plot_heatmap(matrix, title):
    plt.figure()
    plt.imshow(matrix, origin="lower")
    plt.colorbar()
    plt.title(title)
    plt.show()


def plot_everything(json_path):
    data = load_data(json_path)

    coverage_history = data["coverage_history"]
    pheromone_map = np.array(data["final_pheromone_map"])
    priority_map = np.array(data["final_priority_map"])
    observed_mask = np.array(data["observed_mask"])

    print(f"Final coverage: {coverage_history[-1]:.2f}%")

    plot_coverage(coverage_history)

    plot_paths(data)

    plot_heatmap(pheromone_map, "Final Pheromone Map")

    plot_heatmap(priority_map, "Final Priority Map")

    plot_heatmap(observed_mask, "Observed Coverage Mask")


if __name__ == "__main__":
    plot_everything("iaca_run_output.json")
