# Phase 6 (Phase 4.6): One-Leg Contact-Loss / Fault Analysis

## 1. Objective
Quantitatively evaluate the attitude stability, transient deviation, recovery dynamics, and fault-tolerance boundaries of the validated, frozen Phase 4 SISO Proportional-Integral (PI) body-pitch attitude controller when one quadruped leg unexpectedly loses ground contact.

The controller gains ($K_p = 0.50, K_i = 0.20$) remain strictly **frozen** throughout this evaluation. Phase 6 is an assessment and fault-tolerance analysis, not a controller redesign or retuning phase.

---

## 2. Reduced-Order Modeling Principles & Physical Interpretation

### Physical Context of Contact Loss in Quadrupeds
In a quadruped robot, ground reaction forces (GRFs) exerted by stance legs provide the physical moment arm that generates corrective body-pitch torque $T_c(t)$ and counteracts gravity. When one leg unexpectedly loses ground contact (e.g., due to foot slip, sudden depression, or broken contact):
1. **Actuation Authority Reduction**: The robot transitions from a 4-leg (or 2-leg diagonal) stance to a degraded contact polygon, diminishing the maximum feasible net ground reaction moment.
2. **Ground Reaction Asymmetry (Moment Disturbance)**: The sudden absence of upward supporting force on the faulted leg creates an unbalanced gravitational moment arm about the remaining support polygon, inducing an equivalent disturbance torque about the lateral pitch axis.

### Reduced-Order SISO Pitch Representation
Because the project's plant is an authoritative reduced-order single-degree-of-freedom body-pitch model ($J \ddot{\theta} + B \dot{\theta} + K \theta = T_c + T_d$), contact loss is formulated through physically interpretable equivalent reduced-order mechanisms:
1. **Torque Authority Reduction**:
   $$T_{c,\text{available}}(t) = \alpha_i(t) \cdot T_c(t), \quad 0 \le \alpha_i \le 1$$
   where $\alpha = 1.0$ under normal contact, and $\alpha = 0.75$ (nominal 25% torque loss assumption) during one-leg loss.
2. **Contact-Loss Disturbance Torque**:
   $$T_{d,\text{contact}}(t) = \pm T_{\text{fault}}$$
   - **Front-Leg Loss (FL, FR)**: Loss of front upward GRF causes the nose to drop, inducing a **negative (nose-down)** pitch torque disturbance ($T_{d,\text{contact}} = -0.25\text{ N}\cdot\text{m}$).
   - **Rear-Leg Loss (RL, RR)**: Loss of rear upward GRF causes the rear to drop, inducing a **positive (nose-up)** pitch torque disturbance ($T_{d,\text{contact}} = +0.25\text{ N}\cdot\text{m}$).
   - **Lateral Symmetry**: Under the pure SISO pitch channel (without roll dynamics $\phi$), FL and FR produce identical pitch disturbances; likewise, RL and RR are identical. Individual leg identities serve as scenario labels.

---

## 3. Assumptions vs. Research Limitations

### Simulation Scenario Assumptions
1. Single-axis pitch rotation uncoupled from roll ($\phi$) and yaw ($\psi$) body motions.
2. Nominal one-leg contact loss reduces pitch torque authority to $\alpha = 0.75$ and injects an equivalent disturbance magnitude $|T_{d,\text{contact}}| = 0.25\text{ N}\cdot\text{m}$.
3. Recovery tolerance band is defined as $\pm 2\%$ of the setpoint ($\pm 0.02\text{ rad} \approx 1.15^\circ$).

### Important Research Limitations
- **Reduced-Order Approximation**: This simulation represents contact loss via an *equivalent reduced-order SISO torque disturbance and authority reduction*. It does **not** simulate full multi-body ground contact mechanics, foot slip friction cones, or 3D kinematics.
- **Scope of Conclusions**:
  - **What CAN be concluded**: The frozen SISO PI controller possesses inherent integral-action robustness to reject step-like contact asymmetry disturbances up to $0.50\text{ N}\cdot\text{m}$ and can recover from temporary contact loss within $1.2-1.5\text{ s}$ when torque limits are respected.
  - **What CANNOT be concluded**: One cannot claim the physical quadruped is "fully fault-tolerant" in 3D space, because physical contact loss also induces roll tilt and yaw rotation, which require full 3D body attitude control and gait reconfiguration.

---

## 4. Controlled Fault Scenarios

- **Case 0: Nominal Baseline**: Normal 4-leg stance ($\alpha = 1.0, T_{d,\text{contact}} = 0$).
- **Case 1: Front-Left (FL) Temporary Loss**: Onset $t = 3.0\text{ s}$, duration $1.5\text{ s}$, $\alpha = 0.75, T_{d} = -0.25\text{ N}\cdot\text{m}$.
- **Case 2: Front-Right (FR) Temporary Loss**: Symmetrical to FL in pitch axis.
- **Case 3: Rear-Left (RL) Temporary Loss**: Onset $t = 3.0\text{ s}$, duration $1.5\text{ s}$, $\alpha = 0.75, T_{d} = +0.25\text{ N}\cdot\text{m}$.
- **Case 4: Rear-Right (RR) Temporary Loss**: Symmetrical to RL in pitch axis.
- **Case 5: Front-Leg Persistent Contact Loss**: Onset $t = 3.0\text{ s}$, remains uncontacted indefinitely.
- **Case 6: Rear-Leg Persistent Contact Loss**: Onset $t = 3.0\text{ s}$, remains uncontacted indefinitely.
- **Case 7: FL Loss with Actuator Saturation**: Evaluated under Phase 5 torque saturation ($\pm 0.40\text{ N}\cdot\text{m}$).

---

## 5. Fault-Tolerance Classification Criteria

The quantitative severity of contact loss is categorized according to:
- **NORMAL**: Peak pitch deviation $\le 0.05\text{ rad}$ ($\approx 2.86^\circ$), recovery time $\le 1.5\text{ s}$, zero persistent residual error.
- **DEGRADED**: Peak pitch deviation between $0.05\text{ rad}$ and $0.20\text{ rad}$ ($\approx 11.5^\circ$), recovery time $\le 4.0\text{ s}$, system asymptotically stable.
- **CRITICAL**: Peak pitch deviation $> 0.20\text{ rad}$, sustained actuator saturation $> 10\%$ of horizon, or recovery time $> 4.0\text{ s}$.
- **FAILURE**: Closed-loop instability ($\theta(t) \to \infty$) or persistent steady-state offset $> 0.25\text{ rad}$.

---

## 6. Generated Figures & Artifacts

### Exported Figures (`06_RESULTS/figures/`)
1. `phase6_fig01_nominal_pitch_response.png`: Nominal pitch tracking without fault.
2. `phase6_fig02_contact_loss_pitch_response.png`: Pitch response during temporary FL contact loss.
3. `phase6_fig03_reference_vs_actual_pitch.png`: Reference vs actual pitch with recovery tolerance band.
4. `phase6_fig04_tracking_error_during_fault.png`: Tracking error $e(t)$ with fault onset and recontact markers.
5. `phase6_fig05_contact_event_indicator.png`: Discrete leg contact state indicator $c_i(t) \in \{0, 1\}$.
6. `phase6_fig06_contact_disturbance_torque.png`: Equivalent ground-reaction asymmetry disturbance $T_{d,\text{contact}}(t)$.
7. `phase6_fig07_commanded_control_torque.png`: Controller output torque command $u_{\text{cmd}}(t)$.
8. `phase6_fig08_applied_control_torque.png`: Commanded vs actual applied torque showing authority reduction $\alpha$.
9. `phase6_fig09_nominal_vs_fault_comparison.png`: Direct comparison: Nominal vs Front-leg (FL) vs Rear-leg (RL).
10. `phase6_fig10_recovery_tolerance_band.png`: Detailed zoomed-in recovery region within the 2% tolerance band.
11. `phase6_fig11_temporary_vs_persistent_loss.png`: Transient response: Temporary vs Persistent contact loss.
12. `phase6_fig12_four_leg_fault_comparison.png`: Four-leg comparison (FL, FR, RL, RR) showing directional asymmetry and lateral symmetry.
13. `phase6_fig13_severity_sweep_comparison.png`: Pitch responses across torque authority sweep $\alpha \in [0.25, 1.00]$.
14. `phase6_fig14_duration_sweep_comparison.png`: Pitch responses across fault durations $T_{\text{loss}} \in [0.5, 4.0]\text{ s}$.
15. `phase6_fig15_severity_vs_metrics_map.png`: Parametric sensitivity curves: $\alpha$ vs Peak Deviation and Recovery Time.
16. `phase6_fig16_actuator_limits_interaction.png`: Fault recovery under actuator torque saturation ($\pm 0.40\text{ N}\cdot\text{m}$).

### Exported Numerical Tables & Data MAT
- `06_RESULTS/tables/phase6_fault_analysis_summary.csv`: Summary of core scenarios.
- `06_RESULTS/tables/phase6_severity_sweep.csv`: Results across authority factor $\alpha \in [0.2, 1.0]$.
- `06_RESULTS/tables/phase6_duration_sweep.csv`: Results across fault duration $T_{\text{loss}} \in [0.2, 5.0]\text{ s}$.
- `06_RESULTS/exported/phase6_fault_analysis_results.mat`: Full persistent data structure `phase6_results`.

---

## 7. Independent Execution & Reproducibility
Phase 6 is executable independently from any clean MATLAB session:
```matlab
project_setup
run_phase_6
```
or equivalently:
```matlab
run_phase_4_6
```
All simulations are fully deterministic.
