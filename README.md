# Active Suspension Control System using PD & PID Controllers

## Overview

This project presents the design and analysis of an **Active Suspension Control System** using **PD** and **PID** controllers in MATLAB.

The suspension system is modeled as:

[
G(s) = \frac{1}{s^2 + 3s + 2}
]

The objective is to minimize vehicle body vibrations caused by road disturbances while improving:

* Settling time
* Damping behavior
* Ride comfort
* Stability

The project compares:

* Open-loop system
* PD-controlled system
* PID-controlled system

using time-domain and frequency-domain analysis.

---

# Objectives

## Control Goals

* Reduce oscillations caused by road bumps
* Achieve settling time less than 5 seconds
* Improve damping characteristics
* Eliminate steady-state error using PID control

---

# Features

## Implemented Analysis

* Open-loop response analysis
* PD controller design
* PID controller design
* Step response comparison
* Pole-zero analysis
* Bode plot analysis
* Damping ratio evaluation
* Performance metrics visualization

---

# Software Requirements

## Required Software

* MATLAB R2020a or later
* Control System Toolbox

## Optional

* Simulink

---

# System Model

## Transfer Function

[
G(s) = \frac{1}{s^2 + 3s + 2}
]

Where:

* Input = Control force
* Output = Vehicle body displacement
* Disturbance = Road bump (step input)

---

# Controllers Used

## PD Controller

[
C_{PD}(s) = K_p + K_d s
]

Used gains:

* (K_p = 20)
* (K_d = 8)

Purpose:

* Faster response
* Improved damping
* Reduced oscillations

---

## PID Controller

[
C_{PID}(s) = K_p + \frac{K_i}{s} + K_d s
]

Used gains:

* (K_p = 30)
* (K_i = 20)
* (K_d = 10)

Purpose:

* Zero steady-state error
* Better tracking
* Superior ride comfort

---

# MATLAB Code

```matlab
%% Active Suspension Control System - v2 (Fixed & Improved)
% Plant: G(s) = 1 / (s^2 + 3s + 2)
% Controllers: PD and PID
% Goals: Settling time < 5s, minimize oscillations, improve damping

clear; clc; close all;

%% =========================================================
%  1. SYSTEM MODEL
%% =========================================================
G = tf([1], [1 3 2]);

fprintf('=== System Model ===\n');
disp(G);

poles_ol = pole(G);
[wn_ol, zeta_ol] = damp(G);

%% =========================================================
%  2. OPEN-LOOP STEP RESPONSE
%% =========================================================
t = 0:0.01:15;

[y_ol, t_ol] = step(G, t);
info_ol = stepinfo(G);

%% =========================================================
%  3. PD CONTROLLER
%% =========================================================
Kp_pd = 20;
Kd_pd = 8;

C_PD = tf([Kd_pd Kp_pd], [1]);

G_cl_PD = feedback(C_PD * G, 1);

[y_pd, t_pd] = step(G_cl_PD, t);

info_pd = stepinfo(G_cl_PD);

%% =========================================================
%  4. PID CONTROLLER
%% =========================================================
Kp_pid = 30;
Ki_pid = 20;
Kd_pid = 10;

C_PID = tf([Kd_pid Kp_pid Ki_pid], [1 0]);

G_cl_PID = feedback(C_PID * G, 1);

[y_pid, t_pid] = step(G_cl_PID, t);

info_pid = stepinfo(G_cl_PID);

%% =========================================================
%  5. STEP RESPONSE PLOT
%% =========================================================
figure;

plot(t_ol, y_ol, 'b', 'LineWidth', 2);
hold on;

plot(t_pd, y_pd, 'r--', 'LineWidth', 2);

plot(t_pid, y_pid, 'g', 'LineWidth', 2);

grid on;

xlabel('Time (s)');
ylabel('Body Displacement');

title('Active Suspension System Response');

legend('Open Loop', 'PD Controller', 'PID Controller');

%% =========================================================
%  6. BODE PLOT
%% =========================================================
figure;
bode(G, C_PD*G, C_PID*G);
grid on;

%% =========================================================
%  7. POLE-ZERO MAP
%% =========================================================
figure;
pzmap(G_cl_PD, G_cl_PID);
grid on;

%% =========================================================
%  8. PERFORMANCE SUMMARY
%% =========================================================
disp('Open Loop Performance');
disp(info_ol);

disp('PD Controller Performance');
disp(info_pd);

disp('PID Controller Performance');
disp(info_pid);
```

---

# Output Results

## Open-Loop System

* Slower response
* Higher oscillations
* Steady-state value = 0.5

## PD Controller

* Faster settling
* Improved damping
* Reduced oscillations
* Small steady-state error

## PID Controller

* Best overall performance
* Zero steady-state error
* Improved stability
* Better disturbance rejection

---

# Performance Comparison

| Metric             | Open Loop | PD      | PID      |
| ------------------ | --------- | ------- | -------- |
| Settling Time      | Higher    | Reduced | Lowest   |
| Overshoot          | Moderate  | Low     | Very Low |
| Steady-State Error | Present   | Small   | Zero     |
| Ride Comfort       | Poor      | Better  | Best     |

---

# Conclusion

The Active Suspension Control System successfully demonstrates the effectiveness of PD and PID controllers in reducing vehicle vibrations.

The PID controller provides:

* Faster stabilization
* Better damping
* Zero steady-state error
* Improved ride comfort

The designed controllers satisfy the required settling time condition:

[
T_s < 5 \text{ seconds}
]

---

# Author

## Developed for

Control Systems / MATLAB Simulation Project

## Technologies Used

* MATLAB
* Control System Toolbox
* PID/PD Control Design
