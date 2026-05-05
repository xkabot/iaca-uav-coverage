# IACA UAV Coverage: A Reproducibility Study

Reproducibility study and extension of:
> Yedilkhan et al., "Efficient Area Coverage Strategies for High-Altitude UAVs in Smart City Monitoring", *Drones* 2025, 9, 632.

We re-implemented the Inverse Ant Colony Algorithm (IACA) with Artificial Potential Field (APF)
motion control in Webots, reproduced the paper's results, and extended the algorithm with
several improvements described below.

---

## Authors
- **Travis Brown**
- **Andrew Isaacson**
- **Caleb Talbott**

---

## Requirements

- [Webots R2023b or later](https://cyberbotics.com/)
- Python 3.10+
- Python packages: `numpy`, `scipy`, `matplotlib`

Install dependencies:
```bash
pip install numpy scipy matplotlib
```

---

## Project Structure

```
iaca-uav-coverage/
├── worlds/
│   ├── iaca_osm_world.wbt          # Webots world file (Astana, Kazakhstan OSM map)
│   └── iaca_osm_world.wbproj       # Webots project file
├── controllers/
│   ├── shared/
│   │   ├── shared_constants.py     # Global parameters (grid size, world bounds, drone count, etc.)
│   │   └── equations.py            # All equations from the paper, implemented and documented
│   ├── iaca_supervisor/
│   │   ├── iaca_supervisor.py      # Central supervisor: pheromone map, priority map, drone coordination
│   │   └── supervisor_constants.py # Supervisor-specific parameters (P_MAX, alpha, lambda, etc.)
│   └── iaca_drone/
│       ├── iaca_drone.py           # Per-drone controller: APF force, PID stabilization, wind
│       ├── pid_controller.py       # Crazyflie PID controller (provided with drone model)
│       └── drone_constants.py      # Drone-specific parameters (speed, boundary strength, etc.)
├── scripts/
│   ├── plot_iaca_results.py        # Plot coverage over time, drone paths, pheromone/priority maps
│   └── animate_pheromones.py       # Animate pheromone and priority maps as GIFs or live
├── pics/
│   └── **/*.png                    # Images generated from simulations, used in README.md and VERSIONS.md
└── README.md
```

---

## Configuration

All key parameters are set in the constants files. **Start with `shared_constants.py`** for the
most impactful settings:

| Parameter                     | File | Description                                                                                                          |
|-------------------------------|---|----------------------------------------------------------------------------------------------------------------------|
| `NUMBER_OF_DRONES`            | `shared_constants.py` | Number of drones to spawn                                                                                            |
| `MAX_STEPS`                   | `shared_constants.py` | Simulation steps (100k = ~53 min simulated time)                                                                     |
| `GRID_ROWS / GRID_COLS`       | `shared_constants.py` | Grid resolution (paper used 500×500)                                                                                 |
| `WORLD_X/Y_MIN/MAX`           | `shared_constants.py` | Coverage area bounds in meters (paper used ±450m)                                                                    |
| `SENSOR_RADIUS_METERS`        | `shared_constants.py` | Drone sensor coverage radius (we used 10m)                                                                           |
| `SEED`                        | `shared_constants.py` | RNG seed for reproducibility                                                                                         |
| `USE_EXCLUSION`               | `shared_constants.py` | Enable exclusion zones for non-rectangular areas                                                                     |
| `P_MAX`                       | `supervisor_constants.py` | Maximum pheromone value                                                                                              |
| `ALPHA_PHEROMONE`             | `supervisor_constants.py` | Pheromone exponential decay factor                                                                                   |
| `LAMBDA`                      | `supervisor_constants.py` | Spatial pheromone decay factor                                                                                       |
| `MAX_WORLD_SPEED`             | `drone_constants.py` | Max drone speed in m/s                                                                                               |
| `BOUNDARY_STRENGTH`           | `drone_constants.py` | Strength of boundary deterrent force                                                                                 |
| `BOUNDARY_MARGIN`             | `drone_constants.py` | width of area where boundary deterrent force will be applied                                                         |
| `EXCLUSION_MARGIN / STRENGTH` | `drone_constants.py` | Same as boundary variables but for exclusion zones; generally should be larger due to differences in implementations |


> **Note on step count:** Each Webots timestep is 32ms. 15,000 steps ≈ 8 minutes of simulated
> flight; 100,000 steps ≈ 53 minutes.

---

## Running a Simulation

1. Open `worlds/iaca_osm_world.wbt` in Webots.
2. Adjust parameters in `shared_constants.py` as desired.
3. Press **Play** in Webots. The supervisor will automatically spawn the configured number of drones.
4. The simulation will pause automatically when `MAX_STEPS` is reached.
5. Results are saved to `controllers/iaca_supervisor/iaca_run_output.npz`.

---

## Visualizing Results

All scripts should be run from the `scripts/` directory, or with the output `.npz` file in the
same directory.

**Plot coverage over time, drone paths, and final pheromone/priority maps:**
```bash
python plot_iaca_results.py
```

**Animate pheromone and priority maps over time (saves as GIF if `save=True`):**
```bash
python animate_pheromones.py
```

To save GIFs, set `save = True` inside `animate_pheromones.py` and update the output paths.

---

## Results


### Results with Paper's Parameters

The following results are from a full-scale run using the hyper parameters given by the paper: 2 drones, 900×900m area, 500×500 grid,
100,000 steps (~53 minutes simulated flight time), 10m sensor radius (not given in paper, so empirically determined from sensor radius in research).

**Coverage over time**: reaches ~16% by end of run. Note that the paper never reports a
coverage percentage despite this being the primary metric for an area coverage algorithm.

![Coverage Over Time](pics/paper_params/coverage.png)

**Drone paths**: the 2 drones fly with little amounts of overlap, although with only two of them, at the speed they go, they 
do not have very much opportunity to do very much. Even though we are using the hyperparameters given in the paper, including the number of timesteps to let them go for,
they do not go nearly as far as we see them go in the papers figures, which is likely a result of them altering parameters in their implementation without disclosing those changes in the paper.

The drones do stray quite far off the edge of the target area, almost 200m in some cases, which is a major issue with the paper's implementation and a problem we have fixed in our own implementation.

![Drone Paths](pics/paper_params/paths.png)

**Final pheromone map**: only the drones' immediate positions show high pheromone intensity. This is hard to see as both are off the edge of the map, but there is a lack of persistent trail of where they have been, 
which is a major issue with the papers hyperparameters. The pheromone decay is too high and the spatial decay is too aggressive, causing the pheromone to dissipate almost immediately after being deposited.

The near-zero trail left behind is a key issue: without a persistent pheromone trail, drones have little memory of where they've been, which limits coverage efficiency.

![Final Pheromone Map](pics/paper_params/pheromone.png)

**Final priority map**: clear directional gradient pointing toward the least-visited areas,
showing the algorithm is correctly steering drones toward currently un-covered regions. However, the lack of a strong pheromone 
trail means this gradient is almost entirely based on current drone locations rather than a longer-term memory of coverage.

![Final Priority Map](pics/paper_params/priority.png)


### Results with our own Parameters

These results are from a full-scale run using hyper parameters determined by our own expirimentation: 2 drones, 900×900m area, 500×500 grid,
100,000 steps (~53 minutes simulated flight time), 10m sensor radius.

Alpha, lambda, delta max velocity, and max velocity were the core parameters changed to allow us an increased coverage and pheromone memory.

**Coverage over time**: reaches ~24% by end of the run.

![Coverage Over Time](pics/final/coverage.png)

**Drone paths**: the 2 drones fly with some amounts of overlap, which is seen in the paper results as well. Note the lines that 
run closely in parallel are far enough that there is no overlap in coverage due to the 10m sensor radius. 

The drones do not stray far outside the target area, which is due to an alteration in our implementation involving
changes made in how the drones interact when approaching and past the area border, which is a massive improvement over the papers implementation.

![Drone Paths](pics/final/paths.png)

**Final pheromone map**: Our parameters allow for a much stronger and more persistent pheromone trail. This creates a much clearer memory of where drones have been, which helps steer them toward un-covered areas and improves overall coverage efficiency. The trail is still strongest near the drones' current positions, but it extends much farther back along their paths compared to the paper's parameters.

![Final Pheromone Map](pics/final/pheromone.png)

**Final priority map**: Our priority map still shows a clear directional gradient towards the least-visited areas, but with our parameters, this gradient is influenced by a stronger pheromone trail, giving drones better guidance based on both current positions and historical positions.

![Final Priority Map](pics/final/priority.png)


### Animated GIFs of pheromone and priority maps from our Parameters

**Pheromone map animation**:

![Pheromone Map Animation](pics/final/pheromone_animation.gif)

**Priority map animation**:

![Priority Map Animation](pics/final/priority_animation.gif)

---

## Exclusion Zones (Non-Rectangular Areas)

A large issue found with the paper is that the algorithm only supported rectangular coverage areas. This is a major limitation 
for real-world applications, where the area of interest may have an irregular shape or contain obstacles.

To restrict coverage to a non-rectangular area, set `USE_EXCLUSION = True` in
`shared_constants.py`, then define your exclusion bitmap in the `make_exclusion_mask()`
function in `supervisor_constants.py`. Any cell where the mask is `True` is treated as
out-of-bounds: drones will avoid it and it will not count toward coverage percentage.

Example (diamond-shaped coverage area):
```python
def make_exclusion_mask():
    # ... coordinate setup ...
    mask = np.abs(XX) + np.abs(YY) > 500
    return mask
```

The excluded area (red) is avoided effectively, with drones staying well within the
diamond-shaped coverage region:

![Exclusion Zone - Diamond Paths](pics/v4/exclusion-outside-paths.png)

The pheromone and priority maps show how excluded cells are handled; excluded areas are
locked to maximum pheromone and zero priority, plus a repulsive boundary force pushes
drones back toward the valid area:

![Exclusion Zone - Pheromone Map](pics/v4/exclusion-outside-pheromone.png)

![Exclusion Zone - Priority Map](pics/v4/exclusion-outside-priority.png)

Exclusion zones also work for interior areas, such as a no-fly zone in the center of the
map. Drones depart from the center at startup but leave immediately and do not return:

![Exclusion Zone - Middle Excluded](pics/v4/exclusion-middle-paths.png)

The plot below extends past the edge of the target area, confirming that our boundary
deterrent keeps drones very close to the coverage region: a significant improvement
over the original paper's implementation, where drones would consistently fly over 200m out of bounds before turning around:

![Full Size Paths with Out-of-Bounds Shown](pics/v4/full_size_paths.png)

---

## Deviations from the Paper

During reproduction we identified several ambiguities and inconsistencies in the original
paper that required our own judgment calls:

- **Pheromone max value:** The paper states `P_MAX = 220`, but their own pheromone map
  figures show values exceeding 1,000,000. We use `P_MAX = 1,000,000` to match the figures.
- **Coverage sensor:** The paper does not specify a sensor type or coverage radius. We use
  a 10m circular footprint, which we consider realistic for a downward-facing camera at
  low altitude.
- **No coverage percentage reported:** The paper never states what percentage of the area
  was actually covered in any of its runs, despite this being the primary metric for an area coverage algorithm.
- **Webots architecture:** The paper does not describe how logic was split between the
  supervisor and drone controllers. We made our own architectural decisions based on the
  paper's prose descriptions.
- **PID controller:** Not described in the paper. We use the Crazyflie's default PID library
  with additional velocity smoothing to prevent instability during sharp direction changes.
- **Missing hyperparameters:** Several values were not given; others appeared to have been
  changed between the text and figures without disclosure. We tuned `alpha`, `lambda`,
  step size, and drone speed empirically.

---

## Our Extensions

Beyond reproducing the paper, we added the following:

- **Boundary deterrent force** - keeps drones tighter to the target area rather than
  wandering significantly out of bounds before turning around (see `boundary_force()` in
  `iaca_drone.py`)
- **Exclusion zone support** - designate arbitrary areas as off-limits via a boolean bitmap,
  enabling non-rectangular coverage areas (see `make_exclusion_mask()` in
  `supervisor_constants.py`)
- **Dynamic drone spawning** - drone count and spawn positions configured automatically
  from `shared_constants.py` without editing the world file
- **Adaptive neighbor lookup** - scales the number of grid cells examined based on the
  search area size, improving performance on larger maps
- **Corrected pheromone update** - fixed implementation to exactly match the paper's
  equations (Chebyshev distance decay, exponential smoothing)
- **GIF animation output** - visualize how the pheromone and priority maps evolve over
  the course of a full run (see `animate_pheromones.py`)
- **Coverage percentage tracking** - explicitly track and log what percentage of the target
  area has been observed, a metric absent from the original paper


## Further Improvements

- Automated simulation testing via Webots CLI
  - Managed to set this up to run on the software with UI.
  - Slow due to graphics rendering.
- Dynamic λ calculation that scales based on grid size.
- Further hyperparameter optimization.
- Potentially ROS 2 integration for parallel simulations or world batching.
- Testing on larger and more complex environments.
- Comparing against other coverage algorithms (e.g. lawnmower, random walk) for baseline performance.
