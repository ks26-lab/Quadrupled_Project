# Phase 5: Hardware-Realistic SISO Simulation

## 1. Objective
Evaluate the closed-loop performance, stability, and tracking integrity of the frozen Phase 4 SISO Proportional-Integral (PI) body-pitch attitude controller under progressively introduced physical implementation non-idealities. Phase 5 serves as a scientific bridge between idealized continuous-time transfer function design and eventual digital/Simulink/hardware realization.

The controller gains ($K_p = 0.50, K_i = 0.20$) remain strictly **frozen** throughout this evaluation. Phase 5 is an assessment and realism study, not a controller design or re-tuning phase.

---

## 2. Established Project Model vs. Simulation Assumptions

### Established Project Model (Authoritative)
- **Plant Dynamics**: Single-degree-of-freedom body-pitch attitude dynamics:
  $$J \ddot{\theta}(t) + B \dot{\theta}(t) + K \theta(t) = T_c(t) + T_d(t)$$
- **Physical Parameters** (retrieved via `get_robot_parameters.m`):
  - Body pitch moment of inertia ($J$): $0.05 \text{ kg}\cdot\text{m}^2$
  - Pitch damping coefficient ($B$): $0.10 \text{ N}\cdot\text{m}\cdot\text{s/rad}$
  - Restoring stiffness ($K$): $0.00 \text{ N}\cdot\text{m/rad}$
  - Open-loop transfer function: $G(s) = \frac{1}{0.05 s^2 + 0.10 s} = \frac{20}{s(s+2)}$
- **Frozen SISO PI Controller** (loaded from `07_DATA/processed/frozen_pi_controller.mat`):
  $$C(s) = 0.50 + \frac{0.20}{s} = \frac{0.50 s + 0.20}{s}$$
  - Closed-loop poles: $s \approx -0.426, -0.787 \pm 2.964j$ (asymptotically stable).

### Simulation Scenario Assumptions (Not Measured Hardware Specs)
The following parameter values are clearly identified as **simulation scenario assumptions** used to isolate and probe specific non-idealities:
1. **IMU Measurement Noise**: Zero-mean Gaussian white noise with $\sigma = 0.005\text{ rad} \approx 0.286^\circ$, representing typical MEMS IMU attitude estimation uncertainty.
2. **Discrete Sensor Sampling**: $f_{\text{IMU}} = 100\text{ Hz}$ ($T_{s,\text{IMU}} = 0.010\text{ s} = 10\text{ ms}$), representing a standard robotic SPI bus sampling rate.
3. **Measurement Low-Pass Filter**: First-order filter with cutoff frequency $f_c = 15.0\text{ Hz}$ ($\omega_c \approx 94.25\text{ rad/s}$).
4. **Actuator Torque Saturation**: Symmetric peak motor torque limit $u_{\text{max}} = \pm 0.40\text{ N}\cdot\text{m}$ (where unconstrained step peak is $\approx 0.50\text{ N}\cdot\text{m}$).
5. **Actuator Slew Rate Limit**: Slew rate limit $R_{\text{max}} = 5.0\text{ N}\cdot\text{m/s}$, representing motor torque rate-of-rise constraints.
6. **Stance Phase Disturbance**: Torque pulse $T_d = 0.30\text{ N}\cdot\text{m}$ applied from $t = 5.0\text{ s}$ to $5.5\text{ s}$, consistent with the Phase 4.5 disturbance methodology.

---

## 3. Closed-Loop Implementation Architecture
The control loop remains strictly Single-Input Single-Output (SISO):

```
theta_d(t) ───(+)─── e(t) ───[ Discrete PI ]─── u_cmd ───[ Rate Limit ]─── u_rate ───[ Saturation ]─── Tc(t) ──(+)───[ Plant G(s) ]─── theta(t)
               ▲                                                                                          ▲                │
               │                                                                                        Td(t)              ▼
               └──────────────────────[ Low-Pass Filter ] ◄───[ ZOH Sampling ] ◄───(+) ◄───────────────────────────────────┘
                                                                                    ▲
                                                                                 n_theta(t)
```

1. **True Plant Output**: $\theta(t)$ from continuous rigid-body dynamics.
2. **Sensor Noise Injection**: $\theta_{\text{raw}}(t) = \theta(t) + n_\theta(t)$.
3. **Discrete Sampling**: $\theta_{\text{sampled}}[k] = \theta_{\text{raw}}(k \cdot T_{s,\text{IMU}})$ held via Zero-Order Hold.
4. **Signal Conditioning**: Discretized 1st-order low-pass filter: $\theta_f[k] = (1 - \alpha) \theta_f[k-1] + \alpha \theta_{\text{sampled}}[k]$.
5. **Attitude Tracking Error**: $e[k] = \theta_d[k] - \theta_f[k]$.
6. **Discrete PI Controller**:
   - Discrete Euler integrator: $I[k] = I[k-1] + K_i \cdot \Delta t \cdot e[k]$
   - Commanded torque: $u_{\text{cmd}}[k] = K_p \cdot e[k] + I[k]$
7. **Actuator Rate Limiter**: Clamps $|\Delta u / \Delta t| \le R_{\text{max}}$.
8. **Actuator Torque Saturation**: Symmetrically clamps $|u_{\text{actual}}| \le u_{\text{max}}$.
9. **Plant Integration**: Continuous integration via 4th-order Runge-Kutta (RK4) with fine simulation time step $dt = 0.001\text{ s}$.

---

## 4. Controlled Experiment Hierarchy

- **Case 0: Ideal Continuous Baseline**
  - Pure linear continuous closed-loop response. Reproduces Phase 4 benchmark.
- **Case 1: Sensor Noise Only**
  - Evaluates sensitivity to high-frequency IMU stochastic noise ($\sigma = 0.005\text{ rad}$).
- **Case 2: IMU Sampling Only**
  - Evaluates zero-order hold discretization at $100\text{ Hz}$ ($T_s = 10\text{ ms}$).
- **Case 3: Measurement Filtering Only**
  - Evaluates phase lag and delay introduced by the $15\text{ Hz}$ low-pass filter.
- **Case 4: Actuator Saturation Only**
  - Evaluates transient overshoot and rise time when torque is capped at $\pm 0.40\text{ N}\cdot\text{m}$.
- **Case 5: Actuator Rate Limit Only**
  - Evaluates torque slew rate limitation ($R_{\text{max}} = 5.0\text{ N}\cdot\text{m/s}$).
- **Case 6: Combined Realistic Scenario**
  - Simultaneous activation of sensor noise, $100\text{ Hz}$ sampling, $15\text{ Hz}$ filtering, rate limiting, and torque saturation.
- **Case 7: Combined Realistic Scenario with Disturbance**
  - Combined Case 6 system subjected to external pitch disturbance pulse ($T_d = 0.30\text{ N}\cdot\text{m}$ at $t = 5.0-5.5\text{ s}$).
- **Case 8: Anti-Windup Diagnostic**
  - Evaluates integrator windup and conditional clamping under severe saturation ($u_{\text{max}} = 0.30\text{ N}\cdot\text{m}$).

---

## 5. Quantitative Metric Definitions
For every case, the following metrics are evaluated from time-domain simulation data:
- **Rise Time ($t_r$)**: 10% to 90% rise time [s].
- **Settling Time ($t_s$)**: 2% tolerance band settling time [s].
- **Percent Overshoot ($M_p$)**: Maximum peak overshoot percentage [%].
- **Steady-State Error ($e_{\text{ss}}$)**: Mean absolute tracking error over final 20% of the horizon [rad].
- **Peak Error ($e_{\text{peak}}$)**: Maximum absolute tracking error $\max |e(t)|$ [rad].
- **RMS Error ($e_{\text{RMS}}$)**: Root-mean-square tracking error [rad].
- **Maximum Control Effort ($u_{\text{max}}$)**: Peak instantaneous control torque applied to plant [$\text{N}\cdot\text{m}$].
- **RMS Control Effort ($u_{\text{RMS}}$)**: Root-mean-square control torque [$\text{N}\cdot\text{m}$].
- **Percentage Saturated**: Percent of simulation duration with active torque saturation [%].
- **Saturation Duration**: Total cumulative time spent in saturation [s].
- **Rate Limit Events**: Count of transitions into rate-limited slew state.
- **Disturbance Recovery Time**: Time to return to within 2% of setpoint following disturbance removal [s].

---

## 6. Generated Figures & Artifacts

### Exported Figures (`06_RESULTS/figures/`)
1. `phase5_fig01_ideal_vs_noisy_measurement.png`: Ideal pitch vs noisy IMU measurement.
2. `phase5_fig02_true_vs_measured_pitch.png`: True robot pitch state vs measured pitch signal (Case 1).
3. `phase5_fig03_sensor_noise_signal.png`: Stochastic sensor noise signal $n_\theta(t)$.
4. `phase5_fig04_ideal_vs_sampled_response.png`: Ideal response vs $100\text{ Hz}$ ZOH sampled response.
5. `phase5_fig05_filtered_vs_unfiltered_measurement.png`: Raw noisy signal vs $15\text{ Hz}$ low-pass filtered signal.
6. `phase5_fig06_ideal_vs_saturation_response.png`: Transient pitch response under $\pm 0.40\text{ N}\cdot\text{m}$ saturation.
7. `phase5_fig07_commanded_vs_actual_torque.png`: Commanded torque $u_{\text{cmd}}(t)$ vs applied torque $u_{\text{actual}}(t)$.
8. `phase5_fig08_actuator_saturation_indication.png`: Temporal saturation activation window.
9. `phase5_fig09_actuator_rate_limit_indication.png`: Actuator slew rate $|du/dt|$ vs $5.0\text{ N}\cdot\text{m/s}$ limit.
10. `phase5_fig10_ideal_vs_realistic_pitch_response.png`: Ideal linear pitch vs combined realistic pitch response (Case 6).
11. `phase5_fig11_realistic_tracking_error.png`: Tracking error $e(t)$ comparison (Ideal vs Combined Realistic).
12. `phase5_fig12_realistic_control_effort.png`: Control torque $T_c(t)$ comparison (Ideal vs Combined Realistic).
13. `phase5_fig13_combined_realistic_scenario.png`: 3-panel comprehensive summary of Case 6.
14. `phase5_fig14_realistic_scenario_with_disturbance.png`: Pitch response and torques under stance disturbance pulse.
15. `phase5_fig15_metric_comparison_ideal_vs_realistic.png`: Quantitative degradation bar chart.
16. `phase5_fig16_anti_windup_diagnostic.png`: Standard PI vs anti-windup clamping under severe saturation ($u_{\text{max}} = 0.30\text{ N}\cdot\text{m}$).

### Exported Data Files (`06_RESULTS/tables/` & `06_RESULTS/exported/`)
- `06_RESULTS/tables/phase5_performance_comparison.csv`: Numerical metrics for all 8 simulated cases.
- `06_RESULTS/exported/phase5_simulation_results.mat`: Full persistent data structure `phase5_results`.

---

## 7. Assumptions & Limitations

### Assumptions
1. Viscous joint damping is linear ($B = 0.10\text{ N}\cdot\text{m}\cdot\text{s/rad}$) with zero joint friction stiction.
2. The IMU sensor noise is stationary Gaussian white noise.
3. Actuator saturation is purely symmetric ($|u| \le u_{\text{max}}$).
4. Actuator bandwidth dynamics beyond the slew rate limit are assumed sufficiently fast relative to the $3\text{ rad/s}$ closed-loop bandwidth.

### Limitations
1. Does not constitute physical hardware validation (bench testing or embedded deployment required).
2. Multi-axis cross-coupling (roll, yaw, foot contact forces) is omitted from this single-axis body-pitch study.
3. Quantization noise (ADC resolution) is grouped within the effective noise variance $\sigma$.

---

## 8. Reproducibility & Independent Execution
Phase 5 is fully reproducible and executable independently from any clean MATLAB session:
```matlab
project_setup
run_phase_5
```
Stochastic noise generation uses fixed seed `rng(42)`.
