# 3-DOF Hovercraft Control System

## Overview

This project presents the design and implementation of optimal and observer-based controllers for the Quanser 3-DOF Hovercraft platform. The hovercraft is modeled using a six-state linearized state-space representation and controlled using modern state-space techniques.

The experiment demonstrates two advanced controller designs:

- **Part 1:** Linear Quadratic Regulator (LQR)
- **Part 2:** Linear Quadratic Gaussian (LQG)

The LQG controller extends the LQR framework by incorporating a Kalman Filter for optimal state estimation under process and measurement noise.

---

## Objectives

- Develop a linearized state-space model of the hovercraft.
- Verify system controllability and observability.
- Design an optimal LQR controller.
- Design a Kalman Filter observer.
- Implement an LQG controller.
- Validate controller performance through Simulink and experimental testing.

---

## Repository Structure

```text
04_Hovercraft/

├── README.md
│
├── Part_1_LQR/
│   ├── MATLAB/
│   ├── Images/
│   ├── s_hover.mdl
│   └── README.md
│
├── Part_2_LQG/
│   ├── MATLAB/
│   ├── Images/
│   ├── s_hover_lqg_1.slx
│   └── README.md
```

---

## Experimental Workflow

```text
Hovercraft Dynamics
          ↓
State-Space Modeling
          ↓
System Analysis
          ↓
      ┌──────────────┐
      │              │
      │  LQR Design  │
      │              │
      └──────────────┘
              │
              │
      Experimental
       Validation
              │
              ▼
      ┌──────────────┐
      │              │
      │  LQG Design  │
      │              │
      └──────────────┘
              │
       Kalman Filter
              │
              ▼
 Observer-Based Control
              │
              ▼
 Experimental Validation
```

---

# Part 1 — Linear Quadratic Regulator (LQR)

The LQR controller computes an optimal state-feedback gain matrix by minimizing a quadratic cost function involving state deviations and control effort. This provides excellent transient performance and accurate attitude tracking for the hovercraft.

See:

**Part_1_LQR/README.md**

---

# Part 2 — Linear Quadratic Gaussian (LQG)

The LQG controller augments the LQR controller with a Kalman Filter observer to estimate unmeasured system states in the presence of noise. This enables robust observer-based feedback control while preserving optimal control performance.

See:

**Part_2_LQG/README.md**

---

## Controller Comparison

| Feature | LQR | LQG |
|---------|-----|-----|
| Full-State Feedback | ✅ | ✅ |
| Optimal Control | ✅ | ✅ |
| Kalman Filter | ❌ | ✅ |
| State Estimation | ❌ | ✅ |
| Noise Rejection | Limited | Excellent |

---

## Topics Covered

- Hovercraft Dynamics
- State-Space Modeling
- Controllability Analysis
- Observability Analysis
- Linear Quadratic Regulator (LQR)
- Linear Quadratic Gaussian (LQG)
- Kalman Filtering
- Observer Design
- Optimal Control
- Simulink Modeling

---

## Software & Tools

- MATLAB
- Simulink
- Control System Toolbox

---

## Learning Outcomes

- Modeling multi-input multi-output (MIMO) systems
- Designing optimal state-feedback controllers
- Designing Kalman Filters for state estimation
- Implementing observer-based control systems
- Validating advanced controllers using Simulink and experimental hardware