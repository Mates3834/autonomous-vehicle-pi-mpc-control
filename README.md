# Autonomous Vehicle PI-MPC Control

An integrated longitudinal and lateral control framework for autonomous vehicle trajectory tracking using **PI control, Model Predictive Control (MPC), and Pure Pursuit**.

The project combines longitudinal velocity regulation with lateral path-following control and evaluates the resulting closed-loop architecture under different vehicle speeds, road conditions, and road inclinations.

## Overview

The autonomous vehicle control architecture consists of two main control loops:

- **Longitudinal Control:** PI-based velocity tracking
- **Lateral Control:** MPC-based yaw and steering control

A **Pure Pursuit** path-following algorithm generates the reference behavior required by the lateral controller.

The complete framework is implemented and evaluated in **MATLAB/Simulink**.

## Control Architecture

The overall control structure can be summarized as:

```text
                    Reference Path
                         │
                         ▼
                  Pure Pursuit
                         │
                         ▼
               Reference Yaw Rate
                         │
                         ▼
                  MPC Controller
                         │
                    Steering Input
                         │
                         ▼
Reference Speed ──► PI Controller ──► Vehicle Dynamics
                         ▲                  │
                         │                  │
                         └──── Feedback ◄───┘
