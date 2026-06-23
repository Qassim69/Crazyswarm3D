# 🧠 Artificial Potential Field (APF) — Gas Source Localization Logic

This package implements an **Artificial Potential Field (APF)** algorithm for **Gas Source Localization (GSL)** using a swarm of Crazyflie drones. The algorithm guides drones through a combination of:
- 🟢 **Attractive forces** — towards detected gas "bouts" or random search setpoints
- 🔴 **Repulsive forces** — away from walls and other drones

This repository contains two main execution pipelines:

| Pipeline | Files | Use Case |
|---|---|---|
| **Simulation** | `APF_1_1_agent.py` + `APF_1_1_env.py` | High-fidelity ROS 2 SIL simulation |
| **Real-world** | `APF_Field_1_agent.py` + `APF_Field_1_env.py` | Laboratory experiments with physical Crazyflies & SGP30 sensors |

---

## 📋 Table of Contents
1. [🗂️ Workspace Directory Structure](#️-workspace-directory-structure)
2. [🔍 Detailed Script Overviews](#-detailed-script-overviews)
3. [⚙️ Parameters & Tweaking Guide](#️-parameters--tweaking-guide)

---

## 🗂️ Workspace Directory Structure

```
src/gaden_simulation_p/gaden_simulation_p/
├── APF_1_1_agent.py        # Simulation: individual drone control node
├── APF_1_1_env.py          # Simulation: environmental potential field server
├── APF_Field_1_agent.py    # Real-world: individual physical drone control node
└── APF_Field_1_env.py      # Real-world: environmental server + source estimation
```

---

## 🔍 Detailed Script Overviews

### 1. 🖥️ `APF_1_1_agent.py` — Simulation Agent

**Purpose**: Controls a single simulated Crazyflie drone, manages local gas sensor readings, detects bouts, and publishes velocity commands.

**Key Operations**:
- Subscribes to `/mox{id}/Sensor_reading` to receive simulated gas concentration readings.
- Filters gas readings using an **Exponentially Weighted Moving Average (EWMA)** to compute first and second derivatives.
- Detects **bouts** based on zero-crossings of the second derivative and threshold checks on the first derivative.
- Publishes detected bout locations to `GSL/bouts/cf{id}`.
- Requests attractive and repulsive force vectors from the environment server via the `GetForces` service.
- Combines forces into a resultant velocity vector and commands the Crazyflie using `cmdVelocityWorld`.
- Publishes visualization markers (arrows and points) to RViz for debugging.
- Logs all raw sensor data and detected bouts as CSV files inside `log/GSL/CFSim/`.

---

### 2. 🌐 `APF_1_1_env.py` — Simulation Environment Server

**Purpose**: Serves as a centralized physics engine for the potential field simulation. Constructs the map grid and computes all environmental forces.

**Key Operations**:
- Hosts the `GetForces` ROS 2 service.
- Monitors drone positions in the `map` frame using TF2.
- Calculates **repulsive forces** from walls and other drones using analytical potential field models.
- Maintains a **3D grid** of the workspace. When a drone publishes a bout, it drops an attractive "particle" into the grid.
- Diffuses bout particles using a **3D Gaussian filter** to model plume dispersion.
- Computes gradients and converts them into a **vortex (spiraling) vector field** to assist local exploration.
- Normalizes all output forces before returning them to the drone agent.

---

### 3. 🛸 `APF_Field_1_agent.py` — Field / Hardware Agent

**Purpose**: Orchestrates flight control and chemical sensing for a physical Crazyflie drone during laboratory experiments.

**Key Operations**:
- Subscribes to `/cf{id}/sgp30` for dual-channel gas sensor values (Left and Right sensors).
- Instantiates **two independent `BoutDetector` objects** (Left + Right) to double the sensing aperture and plume sensitivity.
- Subscribes to `/cf{id}/battery` — triggers emergency landing and prevents velocity overrides if voltage is critical.
- Integrates a **physical gas source exclusion zone** in the random exploration generator to prevent crashing into the gas source hardware.
- Sends velocity commands via the Crazyflie Python API using **real system clock time** (not simulated clock).
- Logs data into `log/GSL/Field/` using real wall time.

---

### 4. 🔬 `APF_Field_1_env.py` — Field / Hardware Environment Server

**Purpose**: Computes environmental forces and performs real-time gas source localization estimation for real-world experiments.

**Key Operations**:
- Applies a **measurement buffer** (`MEASUREMENT_EPS = 0.10` m) to all distance calculations to account for localization/tracking noise.
- Implements a **hybrid repulsive force profile** combining:
  - Exponential term → close-range safety
  - Linear term → smooth long-range decay
- Calculates a **Gas Source Repulsion Force** to prevent physical drones from colliding with stationary gas release hardware (`X_SOURCE`).
- Runs a **real-time Source Estimation** routine: finds the maximum concentration coordinate (argmax) of the diffused bout map and logs the Euclidean estimation error to `log/GSL/Env/`.

---

## ⚙️ Parameters & Tweaking Guide

The performance of the GSL swarm is highly sensitive to its parameters.

> ⛔ **Critical Parameters** — Do NOT modify without empirical validation. Incorrect values will degrade performance, cause flight instability, or result in physical crashes.

### Critical Parameters

#### 1. 📊 `BOUT_THRESHOLD`

The minimum amplitude change in the smoothed first derivative of the gas sensor signal (`x_d1`) required to confirm a bout.

| Mode | Value | Reason |
|---|---|---|
| Simulation | `200` | High-intensity, noiseless simulated concentration units |
| Real World | `25` | Tuned for SGP30 sensor voltage/resistance scales |

---

#### 2. ⏱️ `TAU`

The half-life (in seconds) of the EWMA smoothing filter. Determines the filter coefficient `alpha`.

| Mode | Value |
|---|---|
| Simulation | `0.5 s` |
| Real World | `0.25 s` |

---

#### 3. 🔒 `CLAMPING_THRESHOLD`

The minimum magnitude of the attractive force vector (`f_bout`) required to affect flight. Values below this threshold are clamped to zero.

| Mode | Value | Reason |
|---|---|---|
| Simulation | `0.25` | Standard threshold for clean simulated fields |
| Real World | `0.675` | Higher to filter spatial interpolation noise in real setups |

---

#### 4. ⚖️ `WEIGHTS`

Relative scaling coefficients between repulsive and attractive forces.

| Mode | Value | Effect |
|---|---|---|
| Simulation | `np.array([2, 1])` | Repulsion weight = 2, Attraction weight = 1 |

> 💡 **Tip**: Increasing the repulsion weight improves collision avoidance at the cost of slower plume convergence. Increasing the attraction weight speeds up source localization but risks collisions in dense swarms.

---

<p align="center">
  📖 Part of the <a href="./README.md">Crazyswarm3D Documentation</a>
</p>
