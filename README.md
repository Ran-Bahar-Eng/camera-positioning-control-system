
# Dynamic Modeling & Control of a Camera Positioning System

An academic control systems design project implementing classical and frequency-domain loop-shaping techniques for precision camera tracking and inverted pendulum stabilization.

---

## Project Overview
This repository contains the dynamic modeling, frequency-domain analysis, and controller synthesis for a DC-motor-driven conveyor belt system:
1. **Fixed Camera Positioning:** Dynamic modeling of motor-belt mechanics, lead-lag compensator design via Loop Shaping, dead-time compensation (Smith Predictor), and tracking harmonic reference signals.
2. **Inverted Pendulum on Cart (Camera on Rod):** Non-linear equation derivation, linearization around stable ($\theta=0$) and unstable ($\theta=\pi$) equilibria, cascade loop control, and stabilization using the Nyquist Stability Criterion.

---

## Key Control Specifications & Results
- **Settling Time:** $t_s < 0.25\text{ s}$ (achieved $0.24\text{ s}$ with Phase Margin of $67.4^\circ$).
- **Overshoot:** $\le 5\%$ (achieved $1.33\%$).
- **Voltage Saturation Constraint:** Maintained strictly below $|U| \le 4.8\text{ V}$.
- **Disturbance Rejection:** Stabilized unstable inverted configuration under impulse disturbance within bounded actuator effort ($|U| \le 12\text{ V}$).

---

## Files
- `*project1.m`: MATLAB simulation scripts for system modeling, loop shaping, and response plots.
- `camera position control.pdf`: Academic project report with analytical derivations

---

*Tel Aviv University, School of Mechanical Engineering - Dynamics & Control of Systems Course.*
