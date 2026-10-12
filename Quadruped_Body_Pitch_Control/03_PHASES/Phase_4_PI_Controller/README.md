# Phase 4: PI Controller Design, Systematic Tuning, Pareto Selection & Controller Freezing

## 1. Objective
Design, systematically evaluate, optimize, select, validate, and freeze a Single-Input Single-Output (SISO) Proportional-Integral (PI) attitude feedback controller for the body-pitch axis of the quadruped robot. The controller must eliminate steady-state error to step commands ($e_{\text{ss}} = 0$) while strictly satisfying transient performance constraints (overshoot, settling time, rise time) and keeping actuator control effort within realistic physical bounds.

---

## 2. Plant Model
The authoritative physical parameters and transfer function are sourced strictly from `02_MODEL/parameters/get_robot_parameters.m` and `02_MODEL/dynamics/get_pitch_plant.m`:

- **Pitch Moment of Inertia ($J$)**: $0.05 \text{ kg}\cdot\text{m}^2$
- **Pitch Damping Coefficient ($B$)**: $0.10 \text{ N}\cdot\text{m}\cdot\text{s/rad}$
- **Restoring Stiffness ($K$)**: $0.00 \text{ N}\cdot\text{m/rad}$ (pure inertial-damping body rotation)

### Open-Loop Transfer Function
$$G(s) = \frac{\theta(s)}{T_c(s)} = \frac{1}{J s^2 + B s + K} = \frac{1}{0.05 s^2 + 0.10 s} = \frac{20}{s(s + 2)}$$

The open-loop plant has poles at $s = 0$ and $s = -2$, possessing type-1 integral behavior.

---

## 3. Controller Formulation
The controller is strictly SISO continuous-time Proportional-Integral:
$$C(s) = K_p + \frac{K_i}{s} = \frac{K_p s + K_i}{s}$$

- **Input**: Tracking error $e(t) = \theta_d(t) - \theta(t)$ [rad]
- **Output**: Net control torque $T_c(t)$ [$\text{N}\cdot\text{m}$]
- **Tuning parameters**: Proportional gain $K_p$, Integral gain $K_i$

### Open-Loop and Closed-Loop Systems
- **Loop Transfer Function**:
  $$L(s) = C(s) G(s) = \frac{K_p s + K_i}{s (0.05 s^2 + 0.10 s)} = \frac{20 (K_p s + K_i)}{s^2 (s + 2)}$$
- **Closed-Loop Transfer Function**:
  $$T(s) = \frac{C(s)G(s)}{1 + C(s)G(s)} = \frac{K_p s + K_i}{0.05 s^3 + 0.10 s^2 + K_p s + K_i}$$
- **Characteristic Equation**:
  $$\Delta(s) = 0.05 s^3 + 0.10 s^2 + K_p s + K_i = 0$$

### Analytical Routh-Hurwitz Stability Criterion
From Routh's array for $\Delta(s) = a_3 s^3 + a_2 s^2 + a_1 s + a_0 = 0$:
$$a_3 = 0.05 > 0, \quad a_2 = 0.10 > 0, \quad a_1 = K_p > 0, \quad a_0 = K_i > 0$$
$$b_1 = \frac{a_2 a_1 - a_3 a_0}{a_2} = \frac{0.10 K_p - 0.05 K_i}{0.10} > 0 \implies K_i < \frac{B}{J} K_p = 2 K_p$$
Therefore, closed-loop asymptotic stability strictly requires:
$$K_p > 0, \quad K_i > 0, \quad K_i < 2 K_p$$

---

## 4. Systematic Tuning Method & Search Grid
A comprehensive 2D parameter grid sweep is conducted across the project's authoritative search domain:
- **$K_p$ range**: $[0.01, 0.50]$ with $\Delta K_p = 0.01$ ($N_{K_p} = 50$)
- **$K_i$ range**: $[0.01, 0.50]$ with $\Delta K_i = 0.01$ ($N_{K_i} = 50$)
- **Total candidates evaluated**: $50 \times 50 = 2,500$ configurations.

---

## 5. Performance Metrics
For every candidate $(K_p, K_i)$:
1. **Stability**: Exact closed-loop poles $p_i = \text{roots}(\Delta(s))$; candidate is stable iff $\max(\text{Re}(p_i)) < -10^{-8}$.
2. **Rise Time ($t_r$)**: Time to transition from 10% to 90% of final steady state.
3. **Settling Time ($t_s$)**: 2% tolerance criterion time ($|y(t) - 1.0| \le 0.02$).
4. **Percent Overshoot ($M_p$)**: $M_p = 100 \times \frac{y_{\text{peak}} - 1.0}{1.0}\%$.
5. **Peak Response ($y_{\text{peak}}$)**: Maximum instantaneous pitch angle [rad].
6. **Steady-State Error ($e_{\text{ss}}$)**: $|1.0 - T(0)|$ (identically 0 due to integral action).
7. **RMS Tracking Error ($e_{\text{RMS}}$)**: $\sqrt{\frac{1}{N}\sum e(t_k)^2}$ over $t \in [0, 10]\text{ s}$.
8. **Maximum Control Effort ($u_{\text{max}}$)**: $\max |u(t)|$ from control sensitivity $T_u(s) = \frac{C(s)}{1 + C(s)G(s)}$.
9. **RMS Control Effort ($u_{\text{RMS}}$)**: $\sqrt{\frac{1}{N}\sum u(t_k)^2}$.

---

## 6. Performance Constraints & Candidate Classification
Candidates are filtered sequentially through rigorous engineering bounds:
- **Stability Constraint**: $\max(\text{Re}(p_i)) < -10^{-8}$ (Mandatory)
- **Overshoot Constraint**: $M_p \le 20.0\%$
- **Settling Time Constraint**: $t_s \le 5.0\text{ s}$
- **Rise Time Constraint**: $t_r \le 2.0\text{ s}$

Candidate funnel progression:
$$\text{Total Candidates (2,500)} \longrightarrow \text{Stable Candidates } (K_i < 2 K_p) \longrightarrow \text{Constraint Feasible Candidates } (M_p \le 20\%, t_s \le 5\text{ s}, t_r \le 2\text{ s})$$

---

## 7. Multi-Objective Pareto Analysis
Pareto trade-off optimization is performed strictly on the **Feasible Candidate Set** across three competing objectives (all minimization):
1. $J_1$: Rise Time ($t_r$)
2. $J_2$: Settling Time ($t_s$)
3. $J_3$: Percent Overshoot ($M_p$)

### Dominance Definition
Candidate $A$ dominates candidate $B$ ($A \prec B$) if and only if:
$$\forall k \in \{1, 2, 3\}: J_k(A) \le J_k(B) \quad \text{and} \quad \exists m \in \{1, 2, 3\}: J_m(A) < J_m(B)$$
The non-dominated candidates form the 3D Pareto frontier.

---

## 8. Deterministic Engineering Selection Rule
From the candidate pool, the final controller is selected via the project's deterministic hierarchical rule:
1. **Mandatory**: Closed-loop asymptotic stability.
2. **Primary**: Strict satisfaction of overshoot target $M_p \le 20\%$.
3. **Secondary**: Minimum settling time $t_s$ among all candidates satisfying $M_p \le 20\%$.
4. **Fallback**: If no candidate satisfies $M_p \le 20\%$, select the candidate with minimal overshoot violation.

---

## 9. Final Controller & Freezing
The selected gains $(K_p^*, K_i^*)$ define the frozen transfer function:
$$C_{\text{final}}(s) = K_p^* + \frac{K_i^*}{s}$$

The frozen controller and complete associated metadata (poles, frequency margins, step metrics, plant parameters) are persisted to:
`07_DATA/processed/frozen_pi_controller.mat`

Variables exported for downstream phases (e.g. Phase 4.5, Simulink):
- `Kp_final`
- `Ki_final`
- `C_final`
- `T_final`
- `info_final`
- `frozen_state` (comprehensive structure)

---

## 10. Generated Figures & Numerical Tables
Execution generates and archives:

### Figures (`06_RESULTS/figures/`):
- `phase4_fig01_stability_map.png`: Numerical stability vs analytical Routh boundary $K_i = 2 K_p$.
- `phase4_fig02_rise_time_map.png`: 2D rise time contour map $t_r(K_p, K_i)$.
- `phase4_fig03_settling_time_map.png`: 2D settling time contour map $t_s(K_p, K_i)$.
- `phase4_fig04_overshoot_map.png`: 2D percent overshoot contour map $M_p(K_p, K_i)$.
- `phase4_fig05_steady_state_error_map.png`: 2D steady-state error map $e_{\text{ss}}(K_p, K_i)$.
- `phase4_fig06_rms_error_map.png`: 2D RMS error map $e_{\text{RMS}}(K_p, K_i)$.
- `phase4_fig07_control_effort_map.png`: 2D maximum control effort map $u_{\text{max}}(K_p, K_i)$.
- `phase4_fig08_feasibility_map.png`: Feasibility map (unstable, non-feasible, fully feasible) with selected point.
- `phase4_fig09_pareto_front.png`: 3D Pareto trade-off space ($t_r, t_s, M_p$).
- `phase4_fig10_final_step_response.png`: Final validated PI step response $\theta(t)$ vs $\theta_d(t)$.
- `phase4_fig11_tracking_error.png`: Validated tracking error $e(t)$.
- `phase4_fig12_control_effort.png`: Validated control effort torque $u(t) = T_c(t)$.
- `phase4_fig13_closed_loop_poles.png`: Final closed-loop pole locations in the $s$-plane.
- `phase4_fig14_p_vs_pi_step_response.png`: Step response comparison between Phase 3 P ($K_p=0.30$) and Phase 4 PI.
- `phase4_fig15_p_vs_pi_metric_comparison.png`: Quantitative performance comparison bar chart.
- `phase4_fig16_bode_stability_margins.png`: Open-loop Bode plot with gain margin $G_m$ and phase margin $P_m$.

### Numerical Tables (`06_RESULTS/tables/`):
- `complete_sweep_results.csv`: All 2,500 candidates with metrics, stability, feasibility, and Pareto status.
- `final_candidate_metrics.csv`: Single-row table with all final metrics and frequency margins.
- `p_vs_pi_comparison.csv`: Direct side-by-side comparison table between baseline P and selected PI.

---

## 11. Assumptions & Limitations
### Assumptions
1. Single-axis pitch rotation uncoupled from roll and yaw dynamics.
2. Linear joint damping ($B = 0.10 \text{ N}\cdot\text{m}\cdot\text{s/rad}$) with no Coulomb friction.
3. Ideal actuator with zero torque delay and linear bandwidth.
4. Unity IMU pitch angle feedback with zero sensor noise.

### Limitations
1. Actuator torque saturation is omitted in Phase 4 (linear analysis). Realistic saturation limits belong to later phases.
2. Integral windup protection (anti-windup) is not included in the linear transfer function formulation.
3. External stance phase disturbances are addressed in Phase 4.5.

---

## 12. Independent Execution
Phase 4 runs completely standalone without requiring prior execution of Phases 1–3:
```matlab
% In MATLAB command window:
project_setup
run_phase_4
```
Or directly from the Phase 4 directory:
```matlab
run_phase_4()
```
