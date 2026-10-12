# Quadruped Robot Body-Pitch Control: Dependency Map

This document details the exact dependency relationships across all phases, scripts, and models in the project as discovered during the project audit.

---

## 1. Global Setup & Path Infrastructure

```
01_SETUP/project_paths.m
    ├── Discovers project root dynamically via mfilename('fullpath')
    └── Adds all canonical subfolders to MATLAB path (00_MAIN, 01_SETUP, 02_MODEL, 03_PHASES, ...)

01_SETUP/project_setup.m
    ├── Invokes project_paths()
    ├── Loads non-control configuration from project_config.m
    └── Verifies all directory structures without modifying parameters or running control logic
```

---

## 2. Model & Parameter Architecture

```
02_MODEL/parameters/robot_parameters.m
    ├── Authoritative Source of Parameters:
    │   ├── J = 0.05 kg.m^2 (Body pitch moment of inertia)
    │   ├── B = 0.10 N.m.s/rad (Pitch damping coefficient)
    │   └── K = 0.00 N.m/rad (Restoring stiffness)
    └── Outputs: Workspace variables J, B, K

02_MODEL/dynamics/pitch_plant.m
    ├── Requires: J, B, K (loads robot_parameters if missing)
    ├── Equations:
    │   ├── G(s) = 1 / (J*s^2 + B*s + K)
    │   └── State-space: A = [0 1; 0 -B/J], B = [0; 1/J], C = [1 0], D = 0
    └── Outputs: Laplace variable s, Transfer function G, State-space sys_ss
```

---

## 3. Phase Dependency Graph

### Phase 1: Physical Model & Initial Response
- **Launcher**: `03_PHASES/Phase_1_Model/run_phase_1.m`
- **Script**: `03_PHASES/Phase_1_Model/phase_1_model.m`
- **Dependencies**:
  - `01_SETUP/project_setup.m`
  - `02_MODEL/parameters/robot_parameters.m`
- **Execution Flow**:
  $$\text{Setup} \longrightarrow \text{Parameters } (J, B, K) \longrightarrow \text{Plant } G(s) \longrightarrow \text{Open-Loop Step} \longrightarrow \text{Initial State Response (Test 3)}$$
- **Outputs**:
  - Open-loop step plot
  - Test 3 initial condition response plot ($\theta_0 = 5^\circ, \omega_0 = 0$)
  - System variables `J, B, K, s, G, sys_ss, theta_initial, t`

---

### Phase 2: SISO Plant & Open-Loop Analysis
- **Launcher**: `03_PHASES/Phase_2_SISO/run_phase_2.m`
- **Script**: `03_PHASES/Phase_2_SISO/phase_2_siso.m`
- **Dependencies**:
  - `01_SETUP/project_setup.m`
  - `02_MODEL/dynamics/pitch_plant.m`
- **Execution Flow**:
  $$\text{Setup} \longrightarrow \text{Plant } G(s) \longrightarrow \text{Poles/Zeros} \longrightarrow \text{Pole-Zero Map} \longrightarrow \text{Step/Impulse Response} \longrightarrow \text{Analytical Validation}$$
- **Outputs**:
  - Poles: $0$ and $-2$
  - Zeros: Empty
  - Type 1 system verification (integrator at origin)
  - Analytical impulse final value: $20/2 = 10.00\text{ rad}$
  - Plots: Pole-Zero Map, Open-Loop Step, Open-Loop Impulse

---

### Phase 3: Feedback Controller Design (Proportional Controller)
- **Launcher**: `03_PHASES/Phase_3_P_Controller/run_phase_3.m`
- **Script**: `03_PHASES/Phase_3_P_Controller/phase_3_p_controller.m`
- **Dependencies**:
  - `01_SETUP/project_setup.m`
  - `02_MODEL/dynamics/pitch_plant.m`
- **Execution Flow**:
  $$\text{Setup} \longrightarrow \text{Plant } G(s) \longrightarrow \text{Nominal } K_p=1 \longrightarrow \text{Parametric Sweep } (K_p \in [0.05, 5]) \longrightarrow \text{Constraint Filtering} \longrightarrow \text{Baseline Comparison } (K_p \in [0.25, 0.30]) \longrightarrow \text{Disturbance Analysis}$$
- **Design Constraints**:
  - Overshoot $\le 25\%$
  - Settling Time $\le 4\text{ s}$
  - Rise Time $\le 1\text{ s}$
- **Outputs**:
  - Results table for $K_p$ sweep
  - Feasible gains table
  - Baseline comparison table ($K_p = 0.25, 0.30$)
  - Transient ($0.1\text{ N}\cdot\text{m}$ pulse $2.0-2.5\text{ s}$) & Sustained ($0.1\text{ N}\cdot\text{m}$ step at $2.0\text{ s}$) disturbance simulations
  - Theoretical vs Simulated disturbance steady-state error table: $e_{\text{ss}} = T_{d0}/K_p$

---

### Phase 4: PI Controller Design, Systematic Tuning, Pareto Selection & Freezing
- **Launcher**: `03_PHASES/Phase_4_PI_Controller/run_phase_4.m`
- **Script**: `03_PHASES/Phase_4_PI_Controller/phase_4_pi_controller.m`
- **Dependencies**:
  - `01_SETUP/project_setup.m`
  - `01_SETUP/project_paths.m`
  - `02_MODEL/parameters/get_robot_parameters.m` (Authoritative parameters $J=0.05, B=0.10, K=0$)
  - `02_MODEL/dynamics/get_pitch_plant.m`
  - `04_COMMON/utilities/print_banner.m`
  - `04_COMMON/utilities/ensure_clean_env.m`
- **Execution Flow**:
  $$\text{Setup} \longrightarrow \text{Authoritative Plant } G(s) \longrightarrow \text{Analytical Routh Boundary } (K_i < 2 K_p) \longrightarrow \text{2D Grid Sweep } (2,500 \text{ candidates: } K_p, K_i \in [0.01, 0.50]) \longrightarrow \text{Poles \& Stability Filter} \longrightarrow \text{Performance Metric Extraction} \longrightarrow \text{Constraint Filter } (M_p \le 20\%, t_s \le 5\text{ s}, t_r \le 2\text{ s}) \longrightarrow \text{3D Pareto Dominance} \longrightarrow \text{Hierarchical Selection} \longrightarrow \text{Independent Validation \& Bode Margins} \longrightarrow \text{P vs PI Comparison} \longrightarrow \text{Controller Freezing} \longrightarrow \text{Export 16 Figures \& 3 Tables}$$
- **Selection Hierarchy**:
  1. Mandatory: Asymptotic stability ($\text{Re}(p) < -10^{-8}$)
  2. Primary: Overshoot $\le 20\%$
  3. Secondary: Minimize settling time
  4. Fallback: Closest candidate to overshoot target
- **Outputs & Artifacts**:
  - Frozen controller `.mat`: `07_DATA/processed/frozen_pi_controller.mat`
    - Variables: `Kp_final`, `Ki_final`, `C_final`, `T_final`, `info_final`, `frozen_state`
  - Exported Numerical Tables (`06_RESULTS/tables/`):
    - `complete_sweep_results.csv` (2,500 candidates with metrics, feasibility, Pareto flags)
    - `final_candidate_metrics.csv`
    - `p_vs_pi_comparison.csv`
  - Exported Figures (`06_RESULTS/figures/`):
    - `phase4_fig01_stability_map.png` through `phase4_fig16_bode_stability_margins.png` (16 total)

---

### Phase 4.5: Disturbance Rejection Validation
- **Launcher**: `03_PHASES/Phase_4_5_Disturbance_Frequency/run_phase_4_5.m`
- **Script**: `03_PHASES/Phase_4_5_Disturbance_Frequency/phase_4_5_disturbance.m`
- **Dependencies**:
  - `01_SETUP/project_setup.m`
  - `02_MODEL/dynamics/pitch_plant.m`
  - `07_DATA/processed/frozen_pi_controller.mat` (or runs Phase 4 candidate selection automatically if unpopulated)
- **Execution Flow**:
  $$\text{Setup} \longrightarrow \text{Plant } G(s) \longrightarrow \text{Frozen PI } (C_{\text{final}}) \longrightarrow G_d(s) = \frac{G}{1+C_{\text{final}}G} \longrightarrow \text{Control Effort } u_{\text{nominal}} \longrightarrow T_d = \max(|u|) \longrightarrow \text{Simulation Horizon} \longrightarrow \text{Tests A, B, C} \longrightarrow \text{Recovery Analysis} \longrightarrow \text{3-Point Validation Check}$$
- **Validation Criteria**:
  1. Theoretical DC disturbance rejection ($G_d(0) = 0$)
  2. Theory vs simulation error within $2\%$ recovery tolerance band
  3. Sustained disturbance recovery within simulation horizon
- **Outputs**:
  - Sustained disturbance response plot
  - Transient disturbance response plot
  - Pitch deviation response plot
  - Final status report: `PHASE 4.5 VALIDATION: PASS`

---

### Phase 5: Hardware-Realistic SISO Simulation
- **Launcher**: `03_PHASES/Phase_5_Hardware_Realistic/run_phase_5.m`
- **Script**: `03_PHASES/Phase_5_Hardware_Realistic/phase_5_hardware_realistic.m`
- **Dependencies**:
  - `01_SETUP/project_setup.m`
  - `01_SETUP/project_paths.m`
  - `02_MODEL/parameters/get_robot_parameters.m` (Authoritative parameters $J=0.05, B=0.10, K=0$)
  - `02_MODEL/dynamics/get_pitch_plant.m`
  - `07_DATA/processed/frozen_pi_controller.mat` (Frozen $K_p=0.50, K_i=0.20$)
  - `04_COMMON/utilities/print_banner.m`
  - `04_COMMON/utilities/ensure_clean_env.m`
- **Execution Flow**:
  $$\text{Setup} \longrightarrow \text{Load Frozen Controller } (K_p, K_i) \longrightarrow \text{Centralized Configuration } (p5\_config) \longrightarrow \text{Case 0: Ideal Baseline} \longrightarrow \text{Case 1: Sensor Noise } (\sigma=0.005\text{ rad}) \longrightarrow \text{Case 2: IMU Sampling } (100\text{ Hz}) \longrightarrow \text{Case 3: Filtering } (15\text{ Hz}) \longrightarrow \text{Case 4: Actuator Saturation } (\pm 0.40\text{ N}\cdot\text{m}) \longrightarrow \text{Case 5: Rate Limiting } (5.0\text{ N}\cdot\text{m/s}) \longrightarrow \text{Case 6: Combined Realistic} \longrightarrow \text{Case 7: Combined + Disturbance} \longrightarrow \text{Case 8: Anti-Windup Clamping} \longrightarrow \text{Metrics Extraction} \longrightarrow \text{Save 16 Figures \& CSV Table}$$
- **Outputs & Artifacts**:
  - Exported Table: `06_RESULTS/tables/phase5_performance_comparison.csv`
  - Exported MAT: `06_RESULTS/exported/phase5_simulation_results.mat`
  - Exported Figures (`06_RESULTS/figures/`):
    - `phase5_fig01_ideal_vs_noisy_measurement.png` through `phase5_fig16_anti_windup_diagnostic.png` (16 total)

---

### Phase 6 (Phase 4.6): One-Leg Contact-Loss / Fault Analysis
- **Launcher**: `03_PHASES/Phase_4_6_Contact_Loss/run_phase_4_6.m` (Alias: `run_phase_6.m`)
- **Script**: `03_PHASES/Phase_4_6_Contact_Loss/phase_4_6_contact_loss.m`
- **Dependencies**:
  - `01_SETUP/project_setup.m`
  - `01_SETUP/project_paths.m`
  - `02_MODEL/parameters/get_robot_parameters.m` (Authoritative parameters $J=0.05, B=0.10, K=0$)
  - `02_MODEL/dynamics/get_pitch_plant.m`
  - `07_DATA/processed/frozen_pi_controller.mat` (Frozen $K_p=0.50, K_i=0.20$)
  - `04_COMMON/utilities/print_banner.m`
  - `04_COMMON/utilities/ensure_clean_env.m`
- **Execution Flow**:
  $$\text{Setup} \longrightarrow \text{Load Frozen Controller } (K_p, K_i) \longrightarrow \text{Case 0: Nominal Baseline} \longrightarrow \text{Cases 1-4: FL, FR, RL, RR Temporary Faults } (\alpha=0.75, T_{d}=\pm 0.25\text{ N}\cdot\text{m}) \longrightarrow \text{Cases 5-6: Persistent Faults} \longrightarrow \text{Case 7: Fault with Saturation} \longrightarrow \text{Severity Sweep } (\alpha \in [0.2, 1.0]) \longrightarrow \text{Duration Sweep } (T_{\text{loss}} \in [0.2, 5.0]\text{ s}) \longrightarrow \text{Critical Scenario ID} \longrightarrow \text{Metrics Extraction} \longrightarrow \text{Save 16 Figures \& 3 CSV Tables}$$
- **Outputs & Artifacts**:
  - Exported Tables (`06_RESULTS/tables/`):
    - `phase6_fault_analysis_summary.csv`
    - `phase6_severity_sweep.csv`
    - `phase6_duration_sweep.csv`
  - Exported MAT: `06_RESULTS/exported/phase6_fault_analysis_results.mat`
  - Exported Figures (`06_RESULTS/figures/`):
    - `phase6_fig01_nominal_pitch_response.png` through `phase6_fig16_actuator_limits_interaction.png` (16 total)

### Phase 7: Robustness, Model Uncertainty & Final SISO Validation (Phase 4.7)
- **Launcher**: `03_PHASES/Phase_4_7_Robustness/run_phase_4_7.m` / `run_phase_7.m` / `00_MAIN/run_phase_7.m`
- **Script**: `03_PHASES/Phase_4_7_Robustness/phase_4_7_robustness.m`
- **Dependencies**:
  - `01_SETUP/project_setup.m`
  - `01_SETUP/project_paths.m`
  - `02_MODEL/parameters/get_robot_parameters.m` (Authoritative parameters $J=0.05, B=0.10, K=0$)
  - `02_MODEL/dynamics/get_pitch_plant.m`
  - `07_DATA/processed/frozen_pi_controller.mat` (Frozen $K_p=0.50, K_i=0.20$)
  - `04_COMMON/utilities/print_banner.m`
  - `04_COMMON/utilities/ensure_clean_env.m`
- **Execution Flow**:
  $$\text{Setup} \longrightarrow \text{Load Frozen Controller } (K_p, K_i) \longrightarrow \text{Centralized Uncertainty Config } (\delta_J, \delta_B, \Delta K) \longrightarrow \text{Single-Parameter Sweeps (J, B, K)} \longrightarrow \text{Combined Grid Search (48 Plants)} \longrightarrow \text{Worst-Case Search \& Sensitivity Ranking} \longrightarrow \text{Disturbance Rejection Under Uncertainty} \longrightarrow \text{Analytical Validation} \longrightarrow \text{Frequency-Domain Margins} \longrightarrow \text{Cross-Phase Check (Phase 5 \& 6)} \longrightarrow \text{Save 16 Figures \& 7 CSV Tables \& MAT}$$
- **Outputs & Artifacts**:
  - Exported Tables (`06_RESULTS/tables/`):
    - `phase7_J_uncertainty_sweep.csv`
    - `phase7_B_uncertainty_sweep.csv`
    - `phase7_K_uncertainty_sweep.csv`
    - `phase7_combined_uncertainty_grid.csv`
    - `phase7_parameter_sensitivity_summary.csv`
    - `phase7_nominal_vs_worst_comparison.csv`
    - `phase7_frequency_margins.csv`
    - `final_project_metrics_summary.csv`
  - Exported MAT: `06_RESULTS/exported/phase7_robustness_results.mat`
  - Exported Figures (`06_RESULTS/figures/`):
    - `phase7_fig01_J_vs_settling_time.png` through `phase7_fig16_nominal_vs_uncertain_responses.png` (16 total)

---

## 4. Master Sequential Scripts

For complete end-to-end execution preserving exact frozen master scripts:
- `00_MAIN/main_phase1_to_3.m` (Alias: `main.m`): Executes Phases 1 through 3 sequentially.
- `00_MAIN/main_phase4_to_4_5.m` (Alias: `PI_Controller.m`): Executes Phases 4.1 through 4.5 sequentially.
- `00_MAIN/run_project.m`: Universal interactive and programmatic launcher.
