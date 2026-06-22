# 🚁 Crazyswarm3D — Swarm Robotics Gas Source Localization (GSL)

> An advanced **ROS 2 (Humble)** workspace integrating **Crazyswarm2** and **GADEN** (Gas Dispersion Simulator) to execute decentralized swarm-based **Gas Source Localization (GSL)** using Crazyflie nano-quadcopters. This repository contains algorithms, simulators, visualization tools, and low-level firmware integrations supporting both **Simulation-in-Loop (SIL)** and **Real-World Experiments**.

---

## 📋 Table of Contents

1. [🔧 Prerequisites](#-prerequisites)
2. [🏗️ Workspace Architecture](#️-workspace-architecture)
3. [📁 Logging Directory Setup](#-logging-directory-setup)
4. [💨 Gas Simulation Generation](#-gas-simulation-generation)
5. [🛸 Experiment 1: Physical Drones](#-experiment-1-physical-drones-experiment)
6. [🖥️ Experiment 2: Simulation Drones](#️-experiment-2-simulation-drones-experiment)
7. [📡 ROS 2 Nodes, Topics & Services](./README_NODES.md)
8. [📧 Contact](#-contact)

---

## 🔧 Prerequisites

To build and run this workspace, you will need the following baseline environment:

| Requirement | Version |
|---|---|
| 🐧 **OS** | Ubuntu 22.04.X LTS |
| 🤖 **Middleware** | ROS 2 Humble |

### 📦 Required GitHub Packages

The following software packages must be installed and configured to support simulation and flight tests:

1. **🤖 Crazyswarm2**
   - **Sources**: [GitHub Repo](https://github.com/IMRCLab/crazyswarm2.git) | [Documentation Site](https://imrclab.github.io/crazyswarm2/)
   - **Why it is required**: Core ROS 2 package handling low-level coordinate frames, tracking transform lookups (LPS), and drone APIs.

2. **⚙️ Crazyflie Firmware (`crazyflie-firmware`)**
   - **Source**: [GitHub Repo](https://github.com/bitcraze/crazyflie-firmware.git)
   - **Why it is required**: STM32 firmware compiled locally for Software-in-the-Loop (SIL) simulation. Enables realistic simulation of onboard estimators (Kalman filters) and flight controllers (PID/Mellinger).

3. **🐍 Crazyflie Python Library (`cflib`)**
   - **Source**: [GitHub Repo](https://github.com/bitcraze/crazyflie-lib-python.git)
   - **Why it is required**: Official Python library for Crazyflie communication. Serves as the interface for sending takeoff, land, and velocity commands over Crazyradio PA.

4. **🖥️ Crazyflie Client (`crazyflie-clients-python`)**
   - **Source**: [GitHub Repo](https://github.com/bitcraze/crazyflie-clients-python.git)
   - **Why it is required**: GUI (`cfclient`) for configuring, monitoring, and flashing Crazyflies. Required for tuning parameters, testing radio links, and validating physical hardware before flight.

5. **💨 GADEN — Gas Dispersion Simulator**
   - **Source**: [GitHub Repo](https://github.com/MAPIRlab/gaden.git)
   - **Why it is required**: 3D simulation engine modelling wind dynamics and gas plume distributions. Generates dynamic chemical concentration fields navigated by the swarm.

---

## 🏗️ Workspace Architecture

The workspace is organized into four major functional pillars:

```
workspace/
├── 🤖 src/crazyswarm2/          # Flight control, swarm capabilities & TF tracking
├── 💨 src/gaden/                # Gas plume/filament modelling & sensor simulation
├── 🧠 src/gaden_simulation_p/   # APF navigation intelligence, params & launch files
└── ⚙️  src/crazyflie-firmware/  # Onboard PID/Mellinger controllers & Kalman estimators
```

| Module | Responsibility |
|---|---|
| `src/crazyswarm2` | Flight control, swarm capabilities, TF tracking for Crazyflie drones |
| `src/gaden` | Gas plume/filament modelling and simulated sensor readings |
| `src/gaden_simulation_p` | Artificial Potential Field (APF) navigation, parameters, and launch files |
| `src/crazyflie-firmware` | High-fidelity onboard PID/Mellinger controllers and Kalman estimators |

---

## 📁 Logging Directory Setup

> ⚠️ **Important:** Complete this step before running any experiments to ensure all CSV data is saved correctly.

### 1️⃣ Create Main Log Directories

```bash
mkdir -p log/GSL/CFSim   # Simulation-in-Loop (SIL) Experiments
mkdir -p log/GSL/Field   # Real-world Field Experiments
mkdir -p log/GSL/Env     # Real-time Source Estimation
```

### 2️⃣ Create Sub-folders for Simulated Experiments

```bash
cd log/GSL/CFSim
mkdir -p Sensor
mkdir -p Bout
```

### 3️⃣ Create Sub-folders for Real-world Experiments

```bash
cd log/GSL/Field
mkdir -p Sensor
mkdir -p Bout
```

### 4️⃣ Set Directory Permissions

```bash
chmod 755 log/GSL/CFSim
chmod 755 log/GSL/Field
chmod 755 log/GSL/CFSim/Sensor
chmod 755 log/GSL/CFSim/Bout
chmod 755 log/GSL/Field/Sensor
chmod 755 log/GSL/Field/Bout
chmod 755 log/GSL/Env
```

---

## 💨 Gas Simulation Generation

> 📝 **Note:** These steps must be completed before running any simulation experiments.

### Step 1 — Run the Preprocessing Launch File

Run `gaden_preproc_launch.py` from `src/gaden/test_env/launch`:

```bash
ros2 launch test_env gaden_preproc_launch.py scenario:=10x6_empty_room
```

📂 **Results saved to:**
- `install/test_env/scenario/10x6_empty_room`
- `install/test_env/scenario/10x6_empty_room/wind_simulations`

### Step 2 — Run the Simulation Launch File

Run `gaden_sim_launch.py` from `src/gaden/test_env/launch`:

```bash
ros2 launch test_env gaden_sim_launch.py
```

📂 **Results saved to:**
- `test_env/scenario/10x6_empty_room/gas_simulations/dynamic/`

---

## 🛸 Experiment 1: Physical Drones Experiment

This configuration runs high-level decentralized search logic on actual **physical Crazyflie drones**. Position tracking uses the **Loco Positioning System (LPS)**.

> 📖 More info on LPS: [Bitcraze LPS Documentation](https://www.bitcraze.io/documentation/system/positioning/)

### 🗂️ Key Files

| File | Description |
|---|---|
| 🚀 **Launch File** | `src/gaden_simulation_p/launch/APF_Field_1.py` & `APF_Field_1_hover_swarm.py` — Coordinates startup of the map transform, physical servers, and agent scripts |
| ⚙️ **Swarm Config** | `src/gaden_simulation_p/config/cfreal.yaml` — Drone IDs, radio URIs, start coordinates, marker structures, and firmware parameters |
| 🧠 **Agent Logic** | `src/gaden_simulation_p/gaden_simulation_p/APF_Field_1_agent.py` — Decentralized controller with EWMA filtering and bout detection |
| 🌐 **Physics Server** | `src/gaden_simulation_p/gaden_simulation_p/APF_Field_1_env.py` — Computes and broadcasts inter-drone and boundary repulsion forces |
| 🐍 **CF Python Wrapper** | `src/crazyswarm2/crazyflie_py/crazyflie_py/crazyflie.py` — High-level API for takeoff, hover, land, etc. |
| 🔌 **C++ Driver Server** | `src/crazyswarm2/crazyflie/src/crazyflie_server.cpp` — Bridge translating ROS 2 topics to radio link packets via Crazyradio PA |

### ▶️ Execution

```bash
ros2 launch gaden_simulation_p APF_Field_1.py backend:=cflib sim:=false
```

---

## 🖥️ Experiment 2: Simulation Drones Experiment

For safety and debugging, this environment uses **high-fidelity Software-in-Loop (SIL)** models. Drones navigate simulated gas plumes generated by GADEN.

### 🗂️ Key Files

| File | Description |
|---|---|
| 🚀 **Launch File** | `src/gaden_simulation_p/launch/APF_1_1.py` & `GSL_hover_swarm.py` — Launches environment maps, gas plumes, simulated sensor nodes, and agent nodes |
| ⚙️ **Swarm Config** | `src/gaden_simulation_p/config/cfrobot.yaml` — Simulated drone IDs, initial positions, and firmware controllers |
| 🧠 **Agent Logic** | `src/gaden_simulation_p/gaden_simulation_p/APF_1_1_agent.py` — Decentralized controller with EWMA filtering and bout detection |
| 🌐 **Physics Server** | `src/gaden_simulation_p/gaden_simulation_p/APF_1_1_env.py` — Computes and broadcasts inter-drone and boundary repulsion forces |
| 🐍 **CF Python Wrapper** | `src/crazyswarm2/crazyflie_py/crazyflie_py/crazyflie.py` — High-level API for takeoff, hover, land, etc. |
| 🔄 **Simulation Coordinator** | `src/crazyswarm2/crazyflie_sim/crazyflie_sim/crazyflie_server.py` — Mimics real drone publisher/subscriber APIs |
| 💻 **SIL Model** | `src/crazyswarm2/crazyflie_sim/crazyflie_sim/crazyflie_sil.py` — Simulates STM32 onboard firmware state estimation and controllers |
| ⚡ **Physics Engine** | `src/crazyswarm2/crazyflie_sim/crazyflie_sim/backend/np.py` — NumPy-based multi-body physics equations for simulated flight |
| 📊 **RViz Visualization** | `src/crazyswarm2/crazyflie_sim/crazyflie_sim/visualization/rviz.py` — Transforms state logs into 3D RViz markers |
| 🔁 **PID Outer Loop** | `src/crazyflie-firmware/src/modules/src/controller/controller_pid.c` — Low-level C PID velocity and attitude-rate loop |
| 📍 **Position Controller** | `src/crazyflie-firmware/src/modules/src/controller/position_controller_pid.c` — Translates desired coordinates into target velocities |
| 🎯 **Attitude Controller** | `src/crazyflie-firmware/src/modules/src/controller/attitude_pid_controller.c` — Computes motor PWM from attitude targets |

### ▶️ Execution

```bash
ros2 launch gaden_simulation_p APF_1_1.py
```

---

## 📧 Contact

| | |
|---|---|
| 👤 **Maintainer** | Mr. Qassim Gazi |
| 📬 **Email** | [qassimgazi06@gmail.com](mailto:qassimgazi06@gmail.com) |

---

<p align="center">
  Made with ❤️ — Swarm Robotics & Gas Source Localization Research
</p>
