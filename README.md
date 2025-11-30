# 🌟 **UR5 Manipulation & HRI Simulation — ROS2 (Humble)**

A complete Python + ROS2 project demonstrating **manipulation**, **kinematics**, **Jacobian torque mapping**, **admittance control**, and **teleoperation** — all fully in simulation.

This repository contains **4 self-contained tasks**, designed to match university HRI/manipulation coursework and real robotics workflows.

---

# 📁 **Repository Overview**

```
ros2_ur5_sim/
│
├── images/                     # Demo GIF + screenshots
├── rviz/                       # RViz config for UR5 visualization
│
├── task1/                      # UR5 joint-motion simulation
│   ├── ur5_joint_motion_node.py
│   ├── planar_arm.py
│   ├── lifting_service_node.py
│   ├── force_to_torque_node.py
│   ├── ...
│
├── task2/                      # 3-DoF planar arm kinematics + IK node
│   └── hri_tasks/
│       ├── ik_node.py
│       ├── planar_arm.py
│       ├── teleoperation.launch.py
│       ├── teleop_params.yaml
│
├── task3/                      # Jacobian, torque mapping, admittance control
│   ├── jacobian_torque_node.py
│   ├── admittance_node.py
│   ├── ik_node.py
│   ├── planar_arm.py
│   └── setup.py / package.xml / tests
│
├── task4_teleop/              # Teleoperation system (simulation only)
│   ├── fsr_sensor.py
│   ├── mapping_node.py
│   └── teleoperation.py
│
└── videos/                     # RViz demo recordings
```

---

# 🟥 **TASK 1 — UR5 Joint-Motion (RViz Simulation)**

### ✔ Features

* UR5 URDF visualization
* Custom RViz layout (`rviz/ur5_hri_simulation_fixed.rviz`)
* Joint-state publishing via Python
* Smooth sinusoidal joint trajectories

### ▶ Run RViz

```bash
rviz2 -d rviz/ur5_hri_simulation_fixed.rviz
```

### ▶ Run Motion Node

```bash
ros2 run hri_manipulation ur5_joint_motion_node
```

### 🎥 Demo

![UR5 Demo](images/ur5_demo.gif)

---

# 🟦 **TASK 2 — 3-DoF Planar Manipulator (FK, IK, ROS2 Nodes)**

A simple 3-DoF arm used to understand FK/IK and ROS2 node interaction.

### 🔹 `planar_arm.py`

Implements:

* Forward kinematics
* Closed-form inverse kinematics
* Link lengths & geometry utilities

### 🔹 `ik_node.py`

ROS2 node that:

* Computes IK for a predefined target
* Publishes joint angles on `/joint_angles`

### ▶ Run

```bash
ros2 run hri_manipulation ik_node
```

---

# 🟩 **TASK 3 — Jacobian Torque Mapping + Admittance Control**

This task simulates the entire *interaction pipeline*:
**joint angles → Jacobian → torque → compliant motion**

### 🔹 `jacobian_torque_node.py`

* Computes analytical Jacobian
* Simulates external force input
* Maps force → joint torques using:
  [
  τ = J^T F
  ]

### 🔹 `admittance_node.py`

Simulates 1-DoF compliant behavior using a mass–spring–damper model:

[
M \ddot{x} + D \dot{x} + K x = F
]

Tracks system position & publishes `/admittance_position`.

### ▶ Run Pipeline

Terminal 1:

```bash
ros2 run hri_manipulation ik_node
```

Terminal 2:

```bash
ros2 run hri_manipulation jacobian_torque_node
```

Terminal 3:

```bash
ros2 run hri_manipulation admittance_node
```

---

# 🟦 **TASK 4 — Teleoperation + Haptic Feedback (Simulation-Only)**

Simulates a remote-operation system **without any hardware**.

### 🔹 `fsr_sensor.py`

Simulates a tactile/force sensor:

* Publishes 0–20 N oscillating signal on `/pub_force`

### 🔹 `mapping_node.py`

Normalizes force into [0, 1]:
[
f_{\text{mapped}} = \frac{f}{\text{max_sensor_force}}
]

### 🔹 `teleoperation.py`

Fully simulated teleop system:

* Generates `Twist` commands (vx, wz)
* Simulates gripper width
* Haptic feedback intensity = mapped force

### ▶ Run Nodes

```bash
ros2 run hri_manipulation fsr_sensor
ros2 run hri_manipulation force_mapping_node
ros2 run hri_manipulation teleoperation_node
```

### 🔄 Data Flow

```
fsr_sensor → pub_force
        ↓
mapping_node → force_mapped
        ↓
teleoperation_node → cmd_vel, gripper_width, haptic feedback
```

---

# 🎯 **Project Learning Outcomes**

By completing these tasks, you gain hands-on experience with:

### 🧠 Manipulator Kinematics

* FK / IK
* Jacobian construction
* Torque mapping

### 🤖 Control & HRI Concepts

* Admittance control
* Compliant motion
* Teleoperation
* Haptic feedback

### 🔧 ROS2 Development Skills

* Pub/Sub
* Parameters
* Launch files
* Packages & setup.py
* Multi-node interaction
* Testing and simulation workflows

