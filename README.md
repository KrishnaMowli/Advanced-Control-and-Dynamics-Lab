# Advanced Control and Dynamics Laboratory

A professional MATLAB and Simulink repository demonstrating the implementation of classical and modern control techniques on real-time laboratory systems. Each experiment includes controller design, MATLAB implementation, Simulink models, experimental validation, and performance analysis.

---

## Repository Overview

This repository contains laboratory experiments performed on Quanser educational platforms as part of an Advanced Control and Dynamics Laboratory.

The objective of these experiments is to design, simulate, and experimentally validate different state-space and classical control techniques on real engineering systems.

Each experiment contains:

- MATLAB implementation
- Simulink model
- Experimental results
- Controller design methodology
- Documentation

---

# Repository Structure

```
Advanced-Control-and-Dynamics-Lab
│
├── 01_Linear_Cart_Pendulum
│   ├── MATLAB
│   ├── Simulink
│   ├── Images
│   └── README.md
│
├── 02_Rotary_Inverted_Pendulum_Qube
│   ├── Part_1_LQR
│   ├── Part_2_Swing_Up
│   ├── README.md
│
├── 03_Gantry
│   ├── Part_1_LQR
│   ├── Part_2_Pole_Placement
│   ├── Simulink
│   └── README.md
│
├── 04_Hovercraft
│   ├── Part_1_LQR
│   ├── Part_2_LQG
│   └── README.md
│
├── LICENSE
└── README.md
```

---

# Experiments Included

| Experiment | Controller |
|------------|------------|
| Linear Cart Pendulum | PV Controller |
| Rotary Inverted Pendulum | LQR |
| Rotary Inverted Pendulum | Swing-Up Control |
| 2-DOF Gantry | LQR |
| 2-DOF Gantry | Pole Placement |
| 3-DOF Hovercraft | LQR |
| 3-DOF Hovercraft | LQG + Kalman Filter |

---

# Experiment 1
## Linear Cart Pendulum

Position and velocity feedback controller designed for trajectory tracking of the Quanser Linear Cart.

### Highlights

- State-space modelling
- Controller gain computation
- Simulink implementation
- Experimental validation

➡ **See:** `01_Linear_Cart_Pendulum`

---

# Experiment 2
## Rotary Inverted Pendulum (QUBE)

The Quanser QUBE Servo 2 experiment consists of two major stages.

### Part 1
- Linear Quadratic Regulator (LQR)

### Part 2
- Energy-based Swing-Up
- Automatic transition to stabilizing controller

➡ **See:** `02_Rotary_Inverted_Pendulum_Qube`

---

# Experiment 3
## Two Degree-of-Freedom Gantry

Position control of a two-axis gantry using state-space controllers.

### Part 1

- LQR Controller

### Part 2

- Pole Placement Controller

➡ **See:** `03_Gantry`

---

# Experiment 4
## Three Degree-of-Freedom Hovercraft

Attitude control of a laboratory hovercraft platform.

### Part 1

- Linear Quadratic Regulator (LQR)

### Part 2

- Linear Quadratic Gaussian (LQG)
- Kalman State Estimator

➡ **See:** `04_Hovercraft`

---

# Control Techniques Demonstrated

- State Feedback Control
- Pole Placement
- Linear Quadratic Regulator (LQR)
- Linear Quadratic Gaussian (LQG)
- Kalman Filtering
- Swing-Up Control
- Position-Velocity Control
- State-Space Modelling
- Experimental System Identification
- Simulink Based Controller Implementation

---

# Software Used

- MATLAB
- Simulink
- Control System Toolbox
- State-Space Toolbox

---

# Hardware Platforms

- Quanser Linear Cart Pendulum
- Quanser Rotary Inverted Pendulum (QUBE)
- Quanser 2-DOF Gantry
- Quanser 3-DOF Hovercraft

---

# Repository Highlights

✔ MATLAB implementations

✔ Simulink models

✔ Experimental validation

✔ Real hardware implementation

✔ Professional documentation

✔ Modular project organization

---

# Future Work

Additional laboratory experiments and advanced controllers will be incorporated as the repository expands, including:

- Model Predictive Control (MPC)
- Robust Control
- Adaptive Control
- Nonlinear Control
- Multi-Agent Control Systems

---

## Author

**Krishna Mowli**

B.Tech Electronics and Communication Engineering  
National Institute of Technology Tiruchirappalli

M.Tech Elecrical Engineering  
Indian Institute of Technology Hyderabad

Areas of Interest

- Control Systems
- Optimal Control
- Robotics
- Embedded Systems
- Analog Electronics
- Semiconductors
- VLSI