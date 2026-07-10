# Swing-Up Control

## Overview

This project implements an energy-based swing-up controller for the Quanser QUBE-Servo 2 Rotary Inverted Pendulum. The controller injects energy into the pendulum until it reaches the upright position, where stabilization is achieved using the LQR controller.

---

## Contents

- MATLAB parameter initialization
- Simulink swing-up implementation
- Controller switching logic
- Experimental response plots

---

## Experimental Results

### Rotary Arm Position Tracking

![Rotary Arm Position](Images/Swingup_rotary_arm_pos.png)

The rotary arm follows the commanded reference while providing sufficient energy for the pendulum to transition from the hanging position to the upright equilibrium.

---

### Pendulum Swing-Up and Stabilization

![Pendulum Angle](Images/Swingup_Pendulum_angle.png)

The energy-based swing-up controller successfully transfers the pendulum from the downward equilibrium to the upright position. Once the pendulum enters the balancing region, the controller switches to the LQR stabilizer, maintaining the pendulum near the unstable equilibrium.

## Files

```text
MATLAB/
    Final_Code_swingup.m

Simulink/
    swingup_student_1.mdl

Images/
```

---

## Workflow

Energy-Based Swing-Up

↓

Pendulum Near Upright

↓

Automatic Switching

↓

LQR Stabilization

---

## Software

- MATLAB
- Simulink
- Stateflow