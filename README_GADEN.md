# 💨 GADEN Simulation & Launch Configuration Guide

This guide provides a detailed, comprehensive walkthrough of the **GADEN (3D Gas Dispersion Simulator for Mobile Robot Olfaction)** packages and configuration files housed within the `test_env` package (`src/gaden/test_env`).

GADEN allows researchers to preprocess environmental CAD models, simulate gas plume dispersion under complex wind flows, play back gas concentrations, and execute co-simulations with mobile robots carrying simulated wind and gas sensors.

---

## 📋 Table of Contents
1. [🗺️ Overview of the Launch Files](#️-overview-of-the-launch-files)
2. [🔬 Launch File Deep-Dive](#-launch-file-deep-dive)
   - [gaden_preproc_launch.py](#a-gaden_preproc_launchpy--preprocessing)
   - [gaden_sim_launch.py](#b-gaden_sim_launchpy--filament-simulation)
   - [gaden_player_launch.py](#c-gaden_player_launchpy--playback--service-server)
3. [🧠 Logic Scripts & Node Architectures](#-logic-scripts--node-architectures)
4. [⚙️ YAML Configurations & Parameter Mappings](#️-yaml-configurations--parameter-mappings)

---

## 🗺️ Overview of the Launch Files

The `test_env` launch folder (`src/gaden/test_env/launch/`) contains four primary launch scripts that handle the full pipeline from raw 3D CAD geometries to multi-agent robotic search simulations:

| Launch Script | Purpose |
|---|---|
| `gaden_preproc_launch.py` | Generates 3D grids, 2D occupancy map, and simulator config |
| `gaden_sim_launch.py` | Runs the gas filament dispersion simulation |
| `gaden_player_launch.py` | Plays back gas/wind data & serves sensor query topics |
| `main_simbot_launch.py` | Path planning & local robot control |

---

## 🔬 Launch File Deep-Dive

### A. `gaden_preproc_launch.py` — Preprocessing

**Purpose**: Converts 3D STL CAD models of rooms/obstacles and raw CFD wind vector fields (CSV files) into structured 3D cubic grid files.

> ⚠️ **Mandatory**: This must be run **before** any GADEN simulation.

**What Happens**:
1. Retrieves the selected `scenario` and `configuration` values.
2. Resolves the shared ROS parameters file `gaden_params.yaml`.
3. Launches the `gaden_preprocessing` node.
4. Parses room boundaries, columns, obstacles, and doors into 3D voxel grids.
5. Interpolates raw wind velocity fields from CFD coordinates into an occupancy-matched grid.

**Referenced YAML Files**:
- [`gaden_params.yaml`](gaden/test_env/ros_params/gaden_params.yaml) — Specifies the dynamic `projectPath` parameter.
- `config.yaml` at `<scenario>/environment_configurations/<configuration>/config.yaml` — Specifies CAD `.stl` filepaths, door outlet locations, unprocessed CFD wind paths, empty point coordinates, and voxel cell resolution (typically `0.1` m).

**Outputs Generated**:

| Output File | Description |
|---|---|
| `OccupancyGrid3D.csv` | 3D grid voxel states: free, obstacle, outlet |
| `occupancy.pgm` & `occupancy.yaml` | 2D slice map for the Nav2 map server |
| `BasicSimScene.yaml` | Boundary and start location file for the `basic_sim` robot simulator |
| Wind sequence CSVs | Cubic-grid-aligned processed wind vectors |

---

### B. `gaden_sim_launch.py` — Filament Simulation

**Purpose**: Runs the main gas dispersion simulator using the Filament Dispersion Model (combining wind advection, turbulent diffusion, and molecular diffusion).

**What Happens**:
1. Declares simulation parameters (e.g., total sim time, execution run rate).
2. Launches `gaden_environment` to load the 3D room structure for RViz.
3. Launches `gaden_filament_simulator` to read wind grids and release virtual filaments from the gas source.
4. Launches `rviz2` with `gaden.rviz` config for real-time visualization.

**Referenced YAML Files**:
- [`gaden_params.yaml`](gaden/test_env/ros_params/gaden_params.yaml) — Passes `projectPath` and `simulationID` to the filament simulator.

---

### C. `gaden_player_launch.py` — Playback & Service Server

**Purpose**: Plays back previously simulated gas dispersal outputs (CSV filament and wind files) in real-time or compressed time, hosting services that allow virtual sensors to sample concentrations.

**What Happens**:
1. Declares arguments for the scenario, configuration, and playback scene.
2. Spawns `rviz2` to show the gas cloud and source geometry.
3. Runs `gaden_environment` to visualize 3D obstacle models.
4. Launches `gaden_player` to load and stream the simulation frames.

**Referenced YAML Files**:
- [`gaden_params.yaml`](gaden/test_env/ros_params/gaden_params.yaml) — Maps parameters to `gaden_player` and `gaden_environment`.
- Playback configuration at `<scenario>/environment_configurations/<config>/scenes/<playback>.yaml` — Details start iterations, looping flags, simulation mapping, and RViz marker display color.

---

## 🧠 Logic Scripts & Node Architectures

When launch files are executed, they invoke ROS 2 nodes implemented in C++ across the `gaden` package:

### 1. 🏗️ `gaden_preprocessing` — `preprocessing`
**Source**: [`preprocessing.cpp`](gaden/gaden_preprocessing/src/preprocessing.cpp)

Reads the scenario's 3D CAD model STL files and voxelizes them into a 3D grid. Voxel states:
- `0` — Free cell
- `1` — Occupied cell (obstacle/wall)
- `2` — Air outlet

Also parses wind vector files, writes them to a matching grid, and generates the 2D occupancy map slice (`occupancy.pgm` / `occupancy.yaml`) and robot simulation workspace (`BasicSimScene.yaml`).

---

### 2. 🌍 `gaden_environment` — `environment`
**Source**: [`environment.cpp`](gaden/gaden_environment/src/environment.cpp)

Loads the room CAD model geometries and `OccupancyGrid3D.csv`, then publishes them to RViz via `visualization_msgs/msg/MarkerArray` topics (`environment_visualization` and `environment_cad_visualization`). Creates a visual 3D representation of the workspace.

---

### 3. 🌬️ `gaden_filament_simulator` — `filament_simulator`
**Source**: [`filament_simulator.cpp`](gaden/gaden_filament_simulator/src/filament_simulator.cpp)

Solves the physical equations of gas dispersion. Spawns virtual **filaments** (parcels of gas concentration) at the release point and updates their coordinates based on:
- **Advection** — Wind velocity vectors sampled from the CFD grid.
- **Turbulent Diffusion** — Random walk displacement matching turbulent intensity.
- **Molecular Diffusion** — Expansion of filament sigma over time.

Filament log files are saved sequentially to the scenario directory.

---

### 4. ▶️ `gaden_player` — `player`
**Source**: [`simulation_player.cpp`](gaden/gaden_player/src/simulation_player.cpp)

Replays saved filament logs frame-by-frame instead of recalculating dispersion in real-time. Publishes the gas cloud as 3D points in RViz (`Gas_Distribution` topic). Hosts two critical ROS 2 services:

| Service | Class | Description |
|---|---|---|
| `/odor_value` | `gaden_msgs/srv/GasPosition` | Resolves gas concentration (ppm) at requested 3D coordinates |
| `/wind_value` | `gaden_msgs/srv/WindPosition` | Resolves wind velocity vector (u, v, w) at requested 3D coordinates |

---

### 5. 🧪 `simulated_gas_sensor` — `simulated_gas_sensor`
**Source**: [`fake_gas_sensor.cpp`](gaden/simulated_gas_sensor/src/fake_gas_sensor.cpp)

Models a PID or MOX point chemical sensor. Resolves the `/sensor_frame` coordinate in the `/map` frame, calls `/odor_value` to retrieve local concentration, adds sensor noise, and publishes values as `olfaction_msgs/msg/GasSensor`.

---

## ⚙️ YAML Configurations & Parameter Mappings

### 1. `gaden_params.yaml`

This global parameter file defines the path structures for all ROS 2 nodes:

```yaml
gaden_preprocessing:
  ros__parameters:
    projectPath: $(var pkg_dir)/scenarios/$(var scenario)/environment_configurations/$(var configuration)

gaden_environment:
  ros__parameters:
    projectPath: $(var pkg_dir)/scenarios/$(var scenario)/environment_configurations/$(var configuration)

gaden_filament_simulator:
  ros__parameters:
    projectPath: $(var pkg_dir)/scenarios/$(var scenario)/environment_configurations/$(var configuration)
    simulationID: $(var simulation)

gaden_player:
  ros__parameters:
    projectPath: $(var pkg_dir)/scenarios/$(var scenario)/environment_configurations/$(var configuration)
    playbackID: "$(var playback)"
```

> 📝 Variables `$(var pkg_dir)`, `$(var scenario)`, `$(var configuration)`, `$(var simulation)`, and `$(var playback)` are passed down from launch file arguments.

---

### 2. Scenario `config.yaml`

Configures environment geometry and wind parameters during preprocessing:

```yaml
# CAD models of the environment (.stl)
models:
  - ../../cad_models/10x6_walls.stl
# CAD model of the outlets (.stl)
outlets_models:
  - "!color [0.86, 0.19, 0.19, 1.00]"
  - ../../cad_models/10x6_door_left.stl
  - ../../cad_models/10x6_door_right.stl
# Path to unprocessed CFD wind CSVs
unprocessed_wind_files: ../../wind_simulations/dynamic/wind_at_cell_centers
# 3D location of a free-space point
empty_point: [0.5, 0.95, 0]
cell_size: 0.1
uniformWind: false
```

---

### 3. Scenario Playback `scene1.yaml`

Used by `gaden_player` to manage playback loops:

```yaml
playback_initial_iteration: 0
playback_loop:
  loop: false
  from: 0
  to: 0
simulations:
  - sim: sim1
    gas_color: [0.4, 0.4, 0.4]
```

---

<p align="center">
  📖 Part of the <a href="./README.md">Crazyswarm3D Documentation</a>
</p>
