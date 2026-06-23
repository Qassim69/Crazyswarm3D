# 🚀 Crazyswarm3D Launch Configuration & Parameters Guide

This directory houses the ROS 2 Python launch scripts that coordinate the startup of the flight servers, map environment parameters, simulated sensor nodes, and path planning agent scripts.

Execution and flight characteristics are configured via two primary YAML files:

| Config File | Experiment Type | Location |
|---|---|---|
| `cfrobot.yaml` | Simulation (SIL) | `src/gaden_simulation_p/config/cfrobot.yaml` |
| `cfreal.yaml` | Real-world (Physical) | `src/gaden_simulation_p/config/cfreal.yaml` |

---

## 📋 Table of Contents
1. [🖥️ Simulation Experiment — `cfrobot.yaml`](#️-simulation-experiment--cfrobotyaml)
   - [APF_1_1.py](#1-apf_1_1py)
   - [GSL_hover_swarm.py](#2-gsl_hover_swarmpy)
2. [🛸 Real-world Experiment — `cfreal.yaml`](#-real-world-experiment--cfreályaml)
   - [APF_Field_1.py](#1-apf_field_1py)
   - [APF_Field_1_hover_swarm.py](#2-apf_field_1_hover_swarmpy)

---

## 🖥️ Simulation Experiment — `cfrobot.yaml`

The parameters in `cfrobot.yaml` govern Software-in-Loop (SIL) flight models, active simulated drones, and default stabilizer settings.

### 1. `APF_1_1.py`

**Purpose**: Orchestrates the high-level path planning simulation.

**Parameters Extracted**:

| Parameter | Description |
|---|---|
| `robots/[cfID]/enabled` | If `true`, spawns the corresponding drone agent and simulated gas sensor |
| `robots/[cfID]/initial_position` | Starting coordinate (e.g. `[3.5, 3.0, 0.0]`) to initialize TF space |

**How it is used**:
- The enabled `crazyflies_ids` and `crazyflies_positions` are passed as launch arguments to the centralized environment node (`APF_1_1_env.py`).
- Agent nodes (`APF_1_1_agent.py`) are spawned sequentially with a **3-second delay** to avoid startup collisions.

---

### 2. `GSL_hover_swarm.py`

**Purpose**: Initializes the simulated flight coordinator.

**Parameters Extracted**:

| Parameter | Description |
|---|---|
| `robots` | Configures active simulated multi-body instances in the physics backend |
| `robot_types` | Configures type-specific details such as battery voltage limits |
| `robot_types/default/firmware_params` | Feeds onboard STM32 firmware parameters for flight characteristics |

**Firmware Parameters Detail**:

| Param | Value | Effect |
|---|---|---|
| `commander.enHighLevel` | `1` | Enables high-level execution calls (takeoff, hover, land) |
| `stabilizer.estimator` | `2` | Uses **Kalman filter** state estimation |
| `stabilizer.controller` | `1` | Selects **PID controller** (Mellinger not tuned for velocity commands) |

**How it is used**:
- The `sim_node` (`crazyflie_server.py`) imports these structures to mock standard firmware services and implements kinematics via the `np.py` physics backend.

---

## 🛸 Real-world Experiment — `cfreal.yaml`

The parameters in `cfreal.yaml` are designed for physical quadcopter flight, including radio channel details, hardware logger topics, LPS configuration, and velocity constraints.

### 1. `APF_Field_1.py`

**Purpose**: Orchestrates the high-level physical swarm experiment.

**Parameters Extracted**:

| Parameter | Description |
|---|---|
| `robots/[cfID]/enabled` | Determines which physical drone agents are active in the workspace |
| `robots/[cfID]/initial_position` | Drone coordinate within the workspace prior to takeoff |

**How it is used**:
- Sequentially launches the physical environment physics node (`APF_Field_1_env.py`) and agent nodes (`APF_Field_1_agent.py`) with corresponding ID list mappings.

---

### 2. `APF_Field_1_hover_swarm.py`

**Purpose**: Initializes the low-level physical Crazyflie radio driver.

**Parameters Extracted**:

| Parameter | Description |
|---|---|
| `robots/uri` | Radio URI for each Crazyflie (e.g. `radio://0/80/2M/E7E7E7E701`; last 2 digits = drone ID number) |
| `firmware_logging/custom_topics/sgp30/vars` | Logs `sgp30.value2L` and `sgp30.value2R` at **20 Hz** |
| `firmware_logging/custom_topics/battery/vars` | Logs `pm.state` at **1 Hz** for battery state-of-charge |
| `dynamicsConfigurations` | Physical safety flight bounds (see table below) |

**Safety Flight Bounds**:

| Parameter | Value |
|---|---|
| `maxXVelocity` / `maxYVelocity` | `2.0 m/s` |
| `maxZVelocity` | `3.0 m/s` |
| `maxRoll` / `maxPitch` | `1.4 rad` (~80° tilt limit) |

**How it is used**:
- Parameters are fed into the C++ `crazyflie_server` node, which sets up radio communication channels, links with LPS, initializes hardware logging, and applies safety checks on incoming agent velocities to prevent crashes.

> 📡 **URI Example**: `radio://0/80/2M/E7E7E7E702` — the last two digits (`02`) correspond to the number printed on the physical drone.

---

<p align="center">
  📖 Part of the <a href="./README.md">Crazyswarm3D Documentation</a>
</p>
