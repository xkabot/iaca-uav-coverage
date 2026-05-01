import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
from matplotlib.animation import FuncAnimation

from supervisor_constants import *


def load_data(path):
    return np.load(path)


def get_bounds(data):
    return {
        "x_min": float(data["world_x_min"]),
        "x_max": float(data["world_x_max"]),
        "y_min": float(data["world_y_min"]),
        "y_max": float(data["world_y_max"]),
    }


def animate_heatmap(data, snapshots_key, title_prefix, colorbar_label, background_path,
                    interval_ms=200, repeat=True, save_path=None, heatmap_alpha=0.75):
    snapshots = data[snapshots_key]

    if len(snapshots) == 0:
        print(f"No {snapshots_key} found.")
        return None

    bounds = get_bounds(data)
    background_img = mpimg.imread(background_path)

    snapshot_steps = data["snapshot_steps"] if "snapshot_steps" in data else None
    snapshot_times = data["snapshot_times"] if "snapshot_times" in data else None

    fig, ax = plt.subplots()

    ax.imshow(
        background_img,
        extent=[bounds["x_min"], bounds["x_max"], bounds["y_min"], bounds["y_max"]],
        origin="upper"
    )

    image = ax.imshow(
        snapshots[0],
        origin="lower",
        extent=[bounds["x_min"], bounds["x_max"], bounds["y_min"], bounds["y_max"]],
        animated=True,
        alpha=heatmap_alpha,
        vmin=np.min(snapshots),
        vmax=np.max(snapshots)
    )

    colorbar = plt.colorbar(image, ax=ax)
    colorbar.set_label(colorbar_label)

    title = ax.set_title(f"{title_prefix} Animation")
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_aspect("equal", adjustable="box")

    def update(frame_idx):
        image.set_array(snapshots[frame_idx])

        if snapshot_steps is not None and snapshot_times is not None:
            step = int(snapshot_steps[frame_idx])
            sim_time = float(snapshot_times[frame_idx])
            title.set_text(f"{title_prefix} | Snapshot {frame_idx} | Step {step} | Time {sim_time:.2f}s")
        elif snapshot_steps is not None:
            step = int(snapshot_steps[frame_idx])
            title.set_text(f"{title_prefix} | Snapshot {frame_idx} | Step {step}")
        else:
            title.set_text(f"{title_prefix} | Snapshot {frame_idx}")

        return image, title

    animation = FuncAnimation(
        fig,
        update,
        frames=len(snapshots),
        interval=interval_ms,
        repeat=repeat,
        blit=False
    )

    if save_path is not None:
        print(f"Saving animation to {save_path}...")
        fps = max(1, 1000 // interval_ms)
        if save_path.endswith(".gif"):
            animation.save(save_path, writer="pillow", fps=fps)
        else:
            animation.save(save_path, writer="ffmpeg", fps=fps)
        print("Done.")

    plt.show()
    return animation


def animate_pheromone(data, background_path, interval_ms=200, repeat=True, save_path=None):
    return animate_heatmap(
        data=data,
        snapshots_key="pheromone_snapshots",
        title_prefix="Pheromone Map",
        colorbar_label="Pheromone Level",
        background_path=background_path,
        interval_ms=interval_ms,
        repeat=repeat,
        save_path=save_path,
        heatmap_alpha=0.75
    )


def animate_priority(data, background_path, interval_ms=200, repeat=True, save_path=None):
    return animate_heatmap(
        data=data,
        snapshots_key="priority_snapshots",
        title_prefix="Priority Map",
        colorbar_label="Priority Value",
        background_path=background_path,
        interval_ms=interval_ms,
        repeat=repeat,
        save_path=save_path,
        heatmap_alpha=0.75
    )


def main():
    data = load_data("iaca_run_output.npz")
    background_path = "../../pics/background.png"

    print(f"Loaded {len(data['pheromone_snapshots'])} pheromone snapshots.")
    print(f"Loaded {len(data['priority_snapshots'])} priority snapshots.")

    save = False

    if save:
        pher_path = "../../pics/v4/pheromone.gif"
        prio_path = "../../pics/v4/priority.gif"
    else:
        pher_path = None
        prio_path = None

    animate_pheromone(data, background_path, interval_ms=150, repeat=True, save_path=pher_path)
    animate_priority(data, background_path, interval_ms=150, repeat=True, save_path=prio_path)


if __name__ == "__main__":
    main()
