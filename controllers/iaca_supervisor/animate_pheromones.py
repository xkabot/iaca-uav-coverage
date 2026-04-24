import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

from supervisor_constants import *


def load_data(path):
    return np.load(path)


def animate_pheromone(data, interval_ms=200, repeat=True, save_path=None):
    pheromone_snapshots = data["pheromone_snapshots"]

    if len(pheromone_snapshots) == 0:
        print("No pheromone snapshots found.")
        return None

    snapshot_steps = data["snapshot_steps"] if "snapshot_steps" in data else None
    snapshot_times = data["snapshot_times"] if "snapshot_times" in data else None

    fig, ax = plt.subplots()
    image = ax.imshow(
        pheromone_snapshots[0],
        origin="lower",
        animated=True
    )
    colorbar = plt.colorbar(image, ax=ax)
    colorbar.set_label("Pheromone Level")

    title = ax.set_title("Pheromone Map Animation")

    def update(frame_idx):
        image.set_array(pheromone_snapshots[frame_idx])

        if snapshot_steps is not None and snapshot_times is not None:
            step = int(snapshot_steps[frame_idx])
            sim_time = float(snapshot_times[frame_idx])
            title.set_text(f"Pheromone Map | Snapshot {frame_idx} | Step {step} | Time {sim_time:.2f}s")
        elif snapshot_steps is not None:
            step = int(snapshot_steps[frame_idx])
            title.set_text(f"Pheromone Map | Snapshot {frame_idx} | Step {step}")
        else:
            title.set_text(f"Pheromone Map | Snapshot {frame_idx}")

        return image, title

    animation = FuncAnimation(
        fig,
        update,
        frames=len(pheromone_snapshots),
        interval=interval_ms,
        repeat=repeat,
        blit=False
    )

    if save_path is not None:
        print(f"Saving pheromone animation to {save_path}...")
        if save_path.endswith(".gif"):
            animation.save(save_path, writer="pillow", fps=1000//interval_ms)
        else:
            animation.save(save_path, writer="ffmpeg", fps=1000//interval_ms)
        print("Done.")

    plt.show()
    return animation


def animate_priority(data, interval_ms=200, repeat=True, save_path=None):
    priority_snapshots = data["priority_snapshots"]

    if len(priority_snapshots) == 0:
        print("No priority snapshots found.")
        return None

    snapshot_steps = data["snapshot_steps"] if "snapshot_steps" in data else None
    snapshot_times = data["snapshot_times"] if "snapshot_times" in data else None

    fig, ax = plt.subplots()
    image = ax.imshow(
        priority_snapshots[0],
        origin="lower",
        animated=True
    )
    colorbar = plt.colorbar(image, ax=ax)
    colorbar.set_label("Priority Value")

    title = ax.set_title("Priority Map Animation")

    def update(frame_idx):
        image.set_array(priority_snapshots[frame_idx])

        if snapshot_steps is not None and snapshot_times is not None:
            step = int(snapshot_steps[frame_idx])
            sim_time = float(snapshot_times[frame_idx])
            title.set_text(f"Priority Map | Snapshot {frame_idx} | Step {step} | Time {sim_time:.2f}s")
        elif snapshot_steps is not None:
            step = int(snapshot_steps[frame_idx])
            title.set_text(f"Priority Map | Snapshot {frame_idx} | Step {step}")
        else:
            title.set_text(f"Priority Map | Snapshot {frame_idx}")

        return image, title

    animation = FuncAnimation(
        fig,
        update,
        frames=len(priority_snapshots),
        interval=interval_ms,
        repeat=repeat,
        blit=False
    )

    if save_path is not None:
        print(f"Saving pheromone animation to {save_path}...")
        if save_path.endswith(".gif"):
            animation.save(save_path, writer="pillow", fps=1000//interval_ms)
        else:
            animation.save(save_path, writer="ffmpeg", fps=1000//interval_ms)
        print("Done.")

    plt.show()
    return animation


def main():
    data = load_data("iaca_run_output.npz")

    print(f"Loaded {len(data['pheromone_snapshots'])} pheromone snapshots.")
    print(f"Loaded {len(data['priority_snapshots'])} priority snapshots.")

    animate_pheromone(data, interval_ms=150, repeat=True, save_path="../../pics/v4/pheromone.gif")
    animate_priority(data, interval_ms=150, repeat=True, save_path="../../pics/v4/priority.gif")


if __name__ == "__main__":
    main()
