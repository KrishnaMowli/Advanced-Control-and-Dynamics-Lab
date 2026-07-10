# 2-DOF Gantry Control System

## Overview

This project presents the implementation of classical and optimal state-feedback control techniques for the Quanser 2-DOF Gantry system. The objective is to accurately control the gantry position along the X and Y axes while minimizing pendulum oscillations during motion.

The experiment consists of two controller design approaches:

- **Part 1:** Pole Placement
- **Part 2:** Linear Quadratic Regulator (LQR)

Both controllers use the same nonlinear Simulink model for validation and hardware implementation.

---

## Objectives

- Develop the state-space model of the gantry system.
- Verify system controllability.
- Design a Pole Placement controller.
- Design an optimal LQR controller.
- Compare controller performance through experimental validation.

---

## Repository Structure

```text
03_Gantry/

├── README.md
│
├── Part_1_Pole_Placement/
│   ├── MATLAB_Code/
│   ├── Images/
│   └── README.md
│
├── Part_2_LQR/
│   ├── MATLAB_Code/
│   ├── Images/
│   └── README.md
│
└── Simulink/
    └── s_2d_gantry.mdl
```

---

## Experimental Workflow

```text
Physical System Modeling
          ↓
State-Space Representation
          ↓
Controllability Analysis
          ↓
Controller Design
      ↙          ↘
Pole Placement    LQR
      ↘          ↙
 Simulink Validation
          ↓
Hardware Implementation
```

---

# Part 1 — Pole Placement

The Pole Placement controller assigns the closed-loop poles to predefined locations, enabling desired transient characteristics such as settling time, overshoot, and damping.

For implementation details and experimental results, see:

**Part_1_Pole_Placement/README.md**

---

# Part 2 — Linear Quadratic Regulator (LQR)

The LQR controller computes the optimal state-feedback gain by minimizing a quadratic performance index, providing an effective trade-off between tracking accuracy and control effort.

For implementation details and experimental results, see:

**Part_2_LQR/README.md**

---

## Shared Simulink Model

Both controller designs use the same Simulink model. Controller behavior is modified by updating the state-feedback gain computed from the corresponding MATLAB script.

---

## Topics Covered

- State-Space Modeling
- Pole Placement
- Linear Quadratic Regulator (LQR)
- Optimal Control
- Controllability Analysis
- Simulink Modeling
- Experimental Validation
- Gantry Control Systems

---

## Software & Tools

- MATLAB
- Simulink
- Control System Toolbox

---

## Learning Outcomes

- Mathematical modeling of multi-axis systems
- Classical state-feedback control
- Optimal control using LQR
- Controllability analysis
- Simulink implementation
- Experimental controller validation