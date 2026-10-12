# Quadruped Robot Gait and Balance Control: Project Manual and Architecture Documentation

> **Operating Manual & Complete Reference Guide**  
> **Course / Engineering Project:** Quadruped Robot Gait and Balance Control  
> **Authoritative Baseline Version:** 1.0.0 (Unified & Reorganized Architecture)  
> **Target Audience:** Undergraduate / Second-Year Robotics & Control Engineering Students  
> **Applicable Environment:** MATLAB R2020a through R2026a (Control System Toolbox; Deep Learning / RL Toolboxes optional)

---

## Table of Contents
1. [Section A — Project Overview & System Architecture](#section-a--project-overview--system-architecture)
2. [Section B — Complete Repository Directory Tree](#section-b--complete-repository-directory-tree)
3. [Section C — MATLAB Requirements & Environment Setup](#section-c--matlab-requirements--environment-setup)
4. [Section D — File-by-File Documentation Table](#section-d--file-by-file-documentation-table)
5. [Section E — Exact MATLAB Execution Guide](#section-e--exact-matlab-execution-guide)
6. [Section F — Phase 8: Learning-Control Benchmark (PI vs. LSTM vs. TD3)](#section-f--phase-8-learning-control-benchmark-pi-vs-lstm-vs-td3)
7. [Section G — Gait-Speed & Actuator Safety Optimization](#section-g--gait-speed--actuator-safety-optimization)
8. [Section H — Simulink Architecture & Simulation Guide](#section-h--simulink-architecture--simulation-guide)
9. [Section I — Results, Artifacts & Output Management](#section-i--results-artifacts--output-management)
10. [Section J — Practical Troubleshooting Guide](#section-j--practical-troubleshooting-guide)
11. [Section K — Recommended Run Order](#section-k--recommended-run-order)
12. [Section L — Practical Verification Checklist](#section-l--practical-verification-checklist)

---

## Section A — Project Overview & System Architecture

### 1. What This Project Does
This project implements the comprehensive modeling, kinematic gait synthesis, feedback balance control, disturbance rejection, and dynamic speed optimization for a 12-DOF (Degrees of Freedom) quadrupedal walking robot.

The central physical challenge addressed by this repository is **body-pitch stabilization during dynamic locomotion**:
- As a quadruped trots across terrain, cyclic foot swing and ground impact impart significant pitching moments on the main chassis.
- Sloped surfaces, ground compliance changes, and leg contact loss introduce sudden external disturbance torques ($T_d$).
- The software system coordinates rhythmic stepping (gait generation), leg inverse kinematics (IK), continuous body-pitch stabilization (SISO pitch feedback), adaptive gait modulation, and comparative machine-learning control.

### 2. Major Software Subsystems

```
                                      ┌──────────────────────────────────────────────────────────┐
                                      │                1. MOTION COMMAND INPUT                   │
                                      │   v_x (forward), v_y (lateral), omega_z (yaw turn rate)  │
                                      └────────────────────────────┬─────────────────────────────┘
                                                                   │
                                                                   ▼
┌──────────────────────────────────────┐             ┌───────────────────────────────────────────┐
│       2. GAIT GENERATION             │             │          3. BALANCE CONTROL               │
│ - Central Pattern Generator (CPG)    │             │ - Pitch Plant Dynamics G(s) = 1/(Js^2+Bs) │
│ - Phase offsets [0.0, 0.5, 0.5, 0.0] │             │ - Frozen Baseline PI: Kp=0.50, Ki=0.20    │
│ - 3D Foot Trajectory (Cycloidal/Sine)│             │ - Anti-windup clamping & saturation       │
│ - 12-DOF Analytical Leg IK Solver    │             │ - Phase 8 ML: LSTM & TD3 Continuous Actor │
└──────────────────┬───────────────────┘             └─────────────────────┬─────────────────────┘
                   │ q_gait [12x1]                                         │ u_pitch [N.m]
                   │                                                       │ (mapped via delta_q)
                   └───────────────────────────┬───────────────────────────┘
                                               │
                                               ▼
                               ┌───────────────────────────────┐
                               │ 4. INTEGRATION & COUPLING     │
                               │ q_cmd = q_gait + delta_q_bal  │
                               │ e(t) = theta_d - theta        │
                               │ Gait Adaptation on Pitch Err  │
                               └───────────────┬───────────────┘
                                               │
                                               ▼
                               ┌───────────────────────────────┐
                               │ 5. ROBOT PLANT & SIMULATION   │
                               │ - Forward kinematics          │
                               │ - Pitch ODE (RK4 integration) │
                               │ - Actuator & current models   │
                               │ - Disturbance & fault pulses  │
                               └───────────────────────────────┘
```

1. **Body-Pitch Balance Control Subsystem (`Quadruped_Body_Pitch_Control/`, `Balance_Control/`):**
   - Models the single-axis body pitch degree of freedom $\theta(t)$ as a second-order rotational plant.
   - Designed through systematic engineering phases: Open-Loop (Phases 1–2), P-Control (Phase 3), 2D Grid & 3D Pareto PI Tuning (Phase 4), Disturbance Rejection (Phase 4.5), Hardware Realism (Phase 5), Leg Fault Analysis (Phase 4.6/6), and Parametric Robustness (Phase 4.7/7).
2. **Gait Movement Subsystem (`Gait_Movement/`):**
   - Implements foot trajectory planning (cycloidal/sinusoidal lift-off and touchdown), stance/swing phase state machines, and analytical 3-DOF per-leg inverse kinematics.
3. **Robot Movement & Kinematics Subsystem (`Robot_Movement/`):**
   - Defines authoritative robot geometry, hip offsets, link lengths ($L_1=0.12\,\text{m}$, $L_2=0.15\,\text{m}$), and joint limit specifications.
4. **Integration Subsystem (`Integration/`):**
   - Bridges gait joint coordinates and pitch torque commands through standardized interfaces: `Robot_State_Interface`, `Balance_Gait_Interface`, `Gait_Balance_Interface`, and `Disturbance_Interface`.
5. **Gait-Speed & Safety Optimization (`Gait_Optimization/`):**
   - Analyzes candidate locomotion speeds against real actuator limits (peak joint velocity, stall torque, battery current draw, mechanical/electrical power, and dynamic pitch stability).
6. **Simulink Simulation Engines (`Simulink/`, `Simulink_Gait/`):**
   - Provides programmatic model builders (`build_gait_model.m`), standalone discrete time execution engines (`run_gait_simulation.m`), and an 11-test verification harness (`validate_gait_model.m`).

### 3. Engineering Distinctions: Modeling vs. Simulation vs. Hardware Deployment
- **Mathematical Modeling:** Continuous-time transfer functions $G(s) = \frac{1}{J s^2 + B s + K}$ and state-space systems $\dot{\mathbf{x}} = \mathbf{A}\mathbf{x} + \mathbf{B}u$ representing idealized physics.
- **Simulation:** Discrete-time numerical integration (Euler or 4th-order Runge-Kutta at $\Delta t = 0.005\,\text{s}$ / 200 Hz) including sensor noise, quantization, actuator saturation ($\pm 0.40\,\text{N}\cdot\text{m}$), rate limits, and ground impact disturbances.
- **Gait Generation & Kinematics:** Purely kinematic Cartesian foot trajectory generator mapped analytically to joint angles $[q_1, q_2, q_3]^T$ per leg without contact force feedback.
- **Integration Layer:** Co-simulation combining gait angles $q_{\text{gait}}$ with dynamic balance offsets $\Delta q_{\text{balance}}$.
- **Hardware Deployment:** Target implementation on microcontrollers/servos. Currently, physical parameters like motor resistance, back-EMF, and gear backlash are modeled as assumptions awaiting physical benchtop identification.

### 4. Established Implementations vs. Experimental Extensions
- **Established & Authoritative Baseline (Frozen):**
  - Physical parameters: $J = 0.05\,\text{kg}\cdot\text{m}^2$, $B = 0.10\,\text{N}\cdot\text{m}\cdot\text{s/rad}$, $K = 0.00\,\text{N}\cdot\text{m/rad}$.
  - Frozen Baseline PI Controller: $K_p = 0.50$, $K_i = 0.20$, torque saturation $u_{\text{sat}} = \pm 0.40\,\text{N}\cdot\text{m}$.
  - Authoritative leg dimensions: $L_1 = 0.12\,\text{m}$ (hip to knee), $L_2 = 0.15\,\text{m}$ (knee to foot), nominal standing clearance $z_0 = -0.22\,\text{m}$.
  - Phased balance studies 1 through 7 and their sequential master scripts (`main_phase1_to_3.m`, `main_phase4_to_4_5.m`).
- **Experimental & Research Extensions:**
  - **Phase 8 Benchmark (`run_phase_8.m`):** Machine learning comparison evaluating Supervised LSTM and Continuous Policy Actor against the frozen PI controller.
  - **Gait-Speed Optimization (`run_gait_speed_optimization.m`):** Multi-speed parametric study (0.064 m/s to 0.500 m/s) assessing servo saturation and battery current feasibility.
  - **Simulink Unified Gait Architecture (`Simulink_Gait/`):** Automated builder and 11-test verification suite linking gait, IK, and pitch stabilization.

---

## Section B — Complete Repository Directory Tree

The directory tree below reflects the exact repository layout on disk:

```
Quadrupled_Project/
├── .git/                                            # Git version control metadata
├── .gitattributes                                  # Git repository attributes
├── main.m                                          # Root backward-compatible alias to 00_MAIN/main_phase1_to_3.m
├── main.asv                                        # MATLAB editor autosave file for main.m
├── PI_Controller.m                                 # Root backward-compatible alias to 00_MAIN/main_phase4_to_4_5.m
├── PI_Controller.asv                               # MATLAB editor autosave file for PI_Controller.m
├── project_setup.m                                 # Root forwarding environment setup script
├── run_project.m                                   # Root forwarding interactive & programmatic launcher
├── QUADRUPED-ROBOT-GAIT-and-BALANCE-CONTROL (1).pdf # Authoritative project literature & reference paper
├── README.md                                       # Primary project operating manual (this document)
│
├── Balance_Control/                                # Balance Control Subsystem Interface & Wrappers
│   ├── Master_Balance.m                            # Balance subsystem launcher (forwards to run_project)
│   ├── Phase_1_Model/run_phase_1.m                 # Launcher forwarding to Quadruped_Body_Pitch_Control Phase 1
│   ├── Phase_2_OpenLoop/run_phase_2.m              # Launcher forwarding to Quadruped_Body_Pitch_Control Phase 2
│   ├── Phase_3_P_Controller/run_phase_3.m          # Launcher forwarding to Quadruped_Body_Pitch_Control Phase 3
│   ├── Phase_4_PI_Controller/run_phase_4.m         # Launcher forwarding to Quadruped_Body_Pitch_Control Phase 4
│   ├── Phase_4_5_Disturbance_Analysis/run_phase_4_5.m # Launcher forwarding to Phase 4.5
│   ├── Phase_4_6_Contact_Loss/run_phase_4_6.m      # Launcher forwarding to Phase 4.6
│   ├── Phase_4_7_Robustness/run_phase_4_7.m        # Launcher forwarding to Phase 4.7
│   ├── Phase_8_Learning_Benchmark/run_phase_8.m    # Launcher forwarding to Phase 8
│   └── Results/README.md                           # Documentation mapping results to 06_RESULTS/
│
├── Gait_Movement/                                  # Gait Trajectory & Kinematics Subsystem
│   ├── Master_Gait.m                               # Standalone procedural gait foundation script
│   ├── README.md                                   # Gait subsystem architectural overview
│   ├── Phase_0_Interface/
│   │   ├── init_gait_state.m                       # Standardized gait state data constructor
│   │   └── phase_1_interface.m                     # Interface definition script
│   ├── Phase_2_Foot_Trajectory/
│   │   └── phase_3_foot_trajectory.m               # Procedural foot swing curve script
│   ├── Phase_3_Inverse_Kinematics/
│   │   ├── phase_4_inverse_kinematics.m            # Procedural IK verification script
│   │   └── solve_leg_ik.m                          # Pure analytical 3-DOF leg IK solver function
│   └── Phase_4_Single_Leg/
│       └── run_phase_5_single_leg.m                # Procedural single-leg trajectory test
│
├── Robot_Movement/                                 # Robot Multibody & Geometry Subsystem
│   ├── Master_Robot.m                              # Standalone procedural multibody dynamics foundation
│   ├── README.md                                   # Robot movement architectural overview
│   └── Robot_Parameters/
│       ├── get_quadruped_geometry.m                # Central authoritative geometry struct accessor
│       └── phase_2_robot_geometry.m                # Procedural geometry validation script
│
├── Integration/                                    # Subsystem Integration & Co-Simulation
│   ├── Balance_Gait_Interface.m                    # Pitch error to gait adaptation calculator
│   ├── Disturbance_Interface.m                     # Standardized disturbance torque/force generator
│   ├── Gait_Balance_Interface.m                    # Joint command combiner (q_cmd = q_gait + delta_q)
│   ├── Master_Integration.m                        # Closed-loop 4-subsystem verification test script
│   ├── Robot_State_Interface.m                     # Telemetry to SISO error extractor
│   └── Results/README.md                           # Documentation of integration outputs
│
├── Gait_Optimization/                              # Speed, Dynamics & Actuator Safety Analysis
│   ├── calculate_gait_dynamics.m                   # Dynamic torque, speed, and power calculator
│   ├── get_dynamics_and_actuator_limits.m          # Central authoritative motor & battery database
│   ├── plot_speed_optimization.m                   # Comparative engineering figure generator
│   ├── run_gait_speed_optimization.m               # Master launcher for speed optimization study
│   └── run_speed_safety_study.m                    # Closed-loop multi-candidate simulation evaluator
│
├── Simulink/                                       # Multi-Subsystem Architectural Specifications
│   └── README.md                                   # Top-level Simulink architecture documentation
│
├── Simulink_Gait/                                  # Unified Gait Simulink Implementation
│   ├── build_gait_model.m                          # Programmatic builder generating Quadruped_Gait_Unified.slx
│   ├── execute_gait_and_optimization_batch.m       # Full batch runner: 11 tests + optimization + builder
│   ├── initialize_gait_model.m                     # Base workspace parameter loader for Simulink
│   ├── README.md                                   # Unified Simulink architecture specification
│   ├── run_gait_simulation.m                       # Discrete simulation execution engine
│   ├── validate_gait_model.m                       # Comprehensive 11-test verification harness
│   └── adapters/                                   # High-speed adapter functions for Simulink blocks
│       ├── balance_adaptation_feedback.m           # Pitch-error-based step height modulation
│       ├── cpg_gait_generator.m                    # 4-leg Trot phase oscillator function
│       ├── four_leg_foot_trajectory.m              # Discrete 3D foot position generator
│       ├── four_leg_foot_trajectory_continuous.m   # Continuous-time foot position generator
│       ├── four_leg_ik_solver.m                    # 12-DOF vector analytical IK wrapper
│       └── gait_balance_coupler.m                  # Block-level joint angle combiner
│
└── Quadruped_Body_Pitch_Control/                   # Core Modularized Pitch Control Subsystem
    ├── README.md                                   # Pitch control subsystem overview & phase breakdown
    ├── 00_MAIN/                                    # Central Master Launchers
    │   ├── main.m                                  # Forwarding wrapper to main_phase1_to_3.m
    │   ├── main_phase1_to_3.m                      # Frozen master script executing Phases 1 to 3
    │   ├── main_phase4_to_4_5.m                    # Frozen master script executing Phases 4 to 4.5
    │   ├── PI_Controller.m                         # Forwarding wrapper to main_phase4_to_4_5.m
    │   ├── run_phase_6.m                           # Root forwarding launcher for Phase 4.6 (Faults)
    │   ├── run_phase_7.m                           # Root forwarding launcher for Phase 4.7 (Robustness)
    │   └── run_project.m                           # Interactive & programmatic menu launcher
    ├── 01_SETUP/                                   # Setup & Dynamic Path Discovery
    │   ├── project_config.m                        # Non-control project metadata & tolerances
    │   ├── project_paths.m                         # Dynamic root detector & path manager
    │   └── project_setup.m                         # Environment initializer & folder verifier
    ├── 02_MODEL/                                   # Physical Parameters & Plant Models
    │   ├── dynamics/
    │   │   ├── get_pitch_plant.m                   # Functional plant accessor returning G(s)
    │   │   └── pitch_plant.m                       # Script defining G(s) and sys_ss
    │   ├── parameters/
    │   │   ├── get_robot_parameters.m              # Functional parameter accessor (J, B, K)
    │   │   └── robot_parameters.m                  # Script defining workspace variables J, B, K
    │   └── validation/
    │       └── validate_plant.m                    # Analytical pole/zero and impulse validation
    ├── 03_PHASES/                                  # Individual Phase Implementations
    │   ├── Phase_1_Model/                          # Phase 1: Physical Model & Test 3 Initial Response
    │   │   ├── phase_1_model.m
    │   │   └── run_phase_1.m
    │   ├── Phase_2_SISO/                           # Phase 2: Open-Loop Analysis, Poles/Zeros
    │   │   ├── phase_2_siso.m
    │   │   └── run_phase_2.m
    │   ├── Phase_3_P_Controller/                   # Phase 3: Proportional Feedback & Gain Sweeps
    │   │   ├── phase_3_p_controller.m
    │   │   └── run_phase_3.m
    │   ├── Phase_4_PI_Controller/                  # Phase 4: PI Tuning, 2D Sweeps, Pareto Freezing
    │   │   ├── phase_4_pi_controller.m
    │   │   ├── README.md
    │   │   └── run_phase_4.m
    │   ├── Phase_4_5_Disturbance_Frequency/        # Phase 4.5: Sustained/Transient Disturbance Rejection
    │   │   ├── phase_4_5_disturbance.m
    │   │   └── run_phase_4_5.m
    │   ├── Phase_4_6_Contact_Loss/                 # Phase 4.6: One-Leg Contact Loss & Fault Analysis
    │   │   ├── phase_4_6_contact_loss.m
    │   │   ├── README.md
    │   │   ├── run_phase_4_6.m
    │   │   └── run_phase_6.m
    │   ├── Phase_4_7_Robustness/                   # Phase 4.7: Model Uncertainty & Parameter Variations
    │   │   ├── phase_4_7_robustness.m
    │   │   ├── README.md
    │   │   ├── run_phase_4_7.m
    │   │   └── run_phase_7.m
    │   ├── Phase_5_Hardware_Realistic/             # Phase 5: Sampling, Quantization, Noise, Saturation
    │   │   ├── phase_5_hardware_realistic.m
    │   │   ├── README.md
    │   │   └── run_phase_5.m
    │   └── Phase_8_Learning_Control_Benchmark/     # Phase 8: PI vs. LSTM vs. TD3 Benchmark
    │       ├── README.md                           # Mathematical benchmark specification
    │       ├── run_phase_8.m                       # Standalone master benchmark launcher
    │       ├── controllers/
    │       │   ├── lstm_pitch_controller.m         # Recurrent LSTM controller constructor
    │       │   ├── lstm_step.m                     # Recurrent forward inference step
    │       │   ├── pi_baseline_controller.m        # Discrete PI constructor with anti-windup
    │       │   ├── pi_step.m                       # Discrete PI execution step
    │       │   ├── td3_pitch_controller.m          # Continuous TD3 Actor constructor
    │       │   └── td3_step.m                      # Deterministic forward Actor step
    │       ├── simulation/
    │       │   ├── compute_benchmark_metrics.m     # Standardized metric extractor (RMSE, IAE, etc.)
    │       │   ├── generate_test_scenarios.m       # 4 strictly disjoint benchmark test scenarios
    │       │   ├── run_benchmark_suite.m           # Multi-controller test harness runner
    │       │   └── simulate_closed_loop.m          # RK4 SISO plant closed-loop simulator
    │       ├── training/
    │       │   ├── generate_training_data.m        # Multi-scenario trajectory generator
    │       │   ├── train_lstm_controller.m         # Sequence gradient trainer (Adam + BPTT)
    │       │   └── train_td3_controller.m          # Policy optimization engine
    │       └── visualization/
    │           ├── export_benchmark_tables.m       # CSV results and summary table exporter
    │           └── plot_comparative_responses.m    # Publication-grade comparative plotter
    ├── 04_COMMON/                                  # Shared Helper Functions & Plotters
    │   ├── functions/
    │   │   ├── format_poles.m                      # Pole display utility
    │   │   └── step_metrics.m                      # Time-domain step response metric extractor
    │   ├── plotting/
    │   │   ├── plot_disturbance_response.m         # Reusable disturbance plot generator
    │   │   └── plot_step_response.m                # Reusable step response plot generator
    │   └── utilities/
    │       ├── ensure_clean_env.m                  # Non-destructive environment initializer
    │       └── print_banner.m                      # Terminal banner formatting tool
    ├── 05_SIMULINK/                                # Pitch Control Simulink Support
    │   ├── data/README.md                          # Description of Simulink data dictionaries
    │   ├── models/README.md                        # Description of pitch Simulink harnesses
    │   └── scripts/simulink_init.m                 # Pre-simulation workspace variable initializer
    ├── 06_RESULTS/                                 # Generated Artifacts (Initially clean)
    │   ├── exported/README.md                      # Target directory for generated .mat workspace files
    │   ├── figures/README.md                       # Target directory for generated .png figures
    │   └── tables/README.md                        # Target directory for generated .csv numerical tables
    ├── 07_DATA/                                    # Datasets & Processed Models
    │   ├── input/                                  # Reserved for input logs and trajectory files
    │   └── processed/README.md                     # Target for frozen_pi_controller.mat
    ├── 08_DOCUMENTATION/                           # Architectural Reports & Literature
    │   ├── dependency_map.md                       # Comprehensive inter-module dependency map
    │   ├── project_structure_report.md             # Repository reorganization audit report
    │   └── README.md                               # Subsystem documentation index
    └── 09_ARCHIVE/                                 # Verbatim Pre-Reorganization Backups
        ├── README.md
        └── pre_reorganization/
            ├── main.asv
            ├── main.m                              # Original monolithic main.m backup
            ├── PI_Controller.asv
            └── PI_Controller.m                     # Original monolithic PI_Controller.m backup
```

---

## Section C — MATLAB Requirements & Environment Setup

### 1. MATLAB Version & Toolbox Prerequisites
- **Recommended Versions:** MATLAB R2020a, R2021a/b, R2022a/b, R2023a/b, R2024a/b, R2025a, or R2026a.
- **Mandatory Toolboxes:**
  - **Control System Toolbox:** Required for transfer function objects (`tf`), state-space representation (`ss`), pole-zero maps (`pzmap`), Bode plots (`bode`), and step response analysis (`stepinfo`, `step`).
- **Optional Toolboxes (Graceful Degradation):**
  - **Simulink:** Only required if building or simulating `.slx` models programmatically via `build_gait_model.m`. The high-fidelity mathematical simulation engines (`run_gait_simulation.m`, `run_speed_safety_study.m`, `Master_Integration.m`) run directly in pure MATLAB code without Simulink.
  - **Deep Learning Toolbox & Reinforcement Learning Toolbox:** *Not required.* The Phase 8 LSTM and TD3 algorithms include fully custom, self-contained native MATLAB implementations with Backpropagation Through Time (BPTT), Adam optimizer, and Policy Gradient rollouts written in pure matrix arithmetic.

### 2. How to Open the Repository & Set Working Directory
1. Launch MATLAB.
2. In the MATLAB Address Bar or Current Folder panel, navigate to the project root directory.
3. Alternatively, in the MATLAB Command Window, execute:
   ```matlab
   % Replace with your actual directory if different:

   ```
4. Verify your current working directory:
   ```matlab
   pwd
   ```
   *Expected output must end in:* `.../Quadrupled_Project`

### 3. How to Check Installed Toolboxes & Version
Run the following commands in the MATLAB Command Window to check your environment:
```matlab
% Display MATLAB Version
v = version;
fprintf('Current MATLAB Version: %s\n', v);

% Check for Control System Toolbox
has_control = license('test', 'Control_Toolbox') && ~isempty(ver('control'));
fprintf('Control System Toolbox Available: %d\n', has_control);

% Check for Simulink
has_simulink = license('test', 'SIMULINK') && ~isempty(ver('simulink'));
fprintf('Simulink Available: %d\n', has_simulink);
```

### 4. How to Initialize the Project Paths
The project features a dynamic root discovery mechanism. Run:
```matlab
project_setup
```
*What this command does:*
- Determines the exact repository location dynamically using `mfilename('fullpath')`.
- Adds all source folders (`00_MAIN`, `01_SETUP`, `02_MODEL`, `03_PHASES`, `04_COMMON`, `Simulink_Gait`, `Gait_Optimization`, `Integration`, etc.) to the MATLAB search path.
- Verifies that required folders exist.
- Prints a startup banner confirming setup completion.
- **Safety Guarantee:** It does **not** alter robot physical parameters, does **not** modify controller gains, and does **not** execute any simulations automatically.

To initialize silently (ideal inside batch scripts):
```matlab
setup_info = project_setup(true);
```

### 5. Resolving Missing-Function & Path Errors
If MATLAB displays `Undefined function or variable '...'`:
1. Ensure the working directory is the project root (`pwd` should show `Quadrupled_Project`).
2. Run `project_setup` to refresh paths.
3. If an alias conflict occurs, clear the in-memory function cache:
   ```matlab
   clear classes; rehash;
   ```

---

## Section D — File-by-File Documentation Table

The table below catalogs every project-owned MATLAB `.m` file across all modules and phases.

| File / Relative Path | Primary Purpose | Required Inputs | Generated Outputs | Key Dependencies | How to Run / Type |
| :--- | :--- | :--- | :--- | :--- | :--- |
| `project_setup.m` | Root forwarding setup launcher | `silent` (optional boolean) | Path structure struct | `01_SETUP/project_setup.m` | `project_setup` |
| `run_project.m` | Root forwarding interactive launcher | `phase_choice` (optional string) | Dispatches phase | `00_MAIN/run_project.m` | `run_project` or `run_project('4')` |
| `main.m` | Backward-compatibility entry alias | None | Runs Phases 1–3 sequentially | `00_MAIN/main_phase1_to_3.m` | `main` |
| `PI_Controller.m` | Backward-compatibility entry alias | None | Runs Phases 4–4.5 sequentially | `00_MAIN/main_phase4_to_4_5.m` | `PI_Controller` |
| **`00_MAIN/`** | | | | | |
| `00_MAIN/run_project.m` | Central interactive/programmatic menu | `choice` (1–11, 'all', 'Q') | Dispatches selected phase | `project_setup.m`, all phase launchers | `run_project('1')`, `run_project('8')`, etc. |
| `00_MAIN/main_phase1_to_3.m` | Frozen master script for Phases 1–3 | None | Step/impulse plots, pole maps, Kp sweep | `pitch_plant.m`, `robot_parameters.m` | `main_phase1_to_3` |
| `00_MAIN/main_phase4_to_4_5.m` | Frozen master script for Phases 4–4.5 | None | 2D/3D Pareto analysis, disturbance plots | `pitch_plant.m`, `robot_parameters.m` | `main_phase4_to_4_5` |
| `00_MAIN/run_phase_6.m` | Forwarding wrapper for Phase 4.6 | None | Executes contact-loss analysis | `Phase_4_6_Contact_Loss/run_phase_4_6.m` | `run_phase_6` |
| `00_MAIN/run_phase_7.m` | Forwarding wrapper for Phase 4.7 | None | Executes robustness analysis | `Phase_4_7_Robustness/run_phase_4_7.m` | `run_phase_7` |
| **`01_SETUP/`** | | | | | |
| `01_SETUP/project_paths.m` | Dynamic path manager | None | `paths` struct | None | `paths = project_paths();` |
| `01_SETUP/project_setup.m` | Folder verifier & path loader | `silent` (boolean) | `setup_info` struct | `project_paths.m`, `project_config.m` | `project_setup(false);` |
| `01_SETUP/project_config.m` | Project metadata & tolerance config | None | `cfg` struct | None | `cfg = project_config();` |
| **`02_MODEL/`** | | | | | |
| `02_MODEL/parameters/robot_parameters.m` | Physical parameter script | None | Workspace vars: `J, B, K` | None | `robot_parameters;` |
| `02_MODEL/parameters/get_robot_parameters.m` | Functional parameter accessor | None | Struct: `params.J, params.B, params.K` | `robot_parameters.m` | `p = get_robot_parameters();` |
| `02_MODEL/dynamics/pitch_plant.m` | Physical plant dynamics script | Workspace `J, B, K` | Workspace: `s, G, sys_ss` | `robot_parameters.m` | `pitch_plant;` |
| `02_MODEL/dynamics/get_pitch_plant.m` | Functional plant accessor | None | Plant struct: `plant.G, plant.sys_ss` | `pitch_plant.m` | `plant = get_pitch_plant();` |
| `02_MODEL/validation/validate_plant.m` | Analytical validation of plant poles | None | Validation status printout | `pitch_plant.m` | `validate_plant();` |
| **`03_PHASES/`** | | | | | |
| `Phase_1_Model/run_phase_1.m` | Standalone launcher: Phase 1 | None | Step response & initial state response | `phase_1_model.m` | `run_phase_1` |
| `Phase_1_Model/phase_1_model.m` | Execution script: Physical Model | None | Plots: Open-loop step, Test 3 ($5^\circ$ IC) | `robot_parameters.m` | Executed by `run_phase_1` |
| `Phase_2_SISO/run_phase_2.m` | Standalone launcher: Phase 2 | None | Pole-zero map, impulse analytical checks | `phase_2_siso.m` | `run_phase_2` |
| `Phase_2_SISO/phase_2_siso.m` | Execution script: Open-Loop Analysis | None | Type 1 check, poles at $0, -2$ | `pitch_plant.m` | Executed by `run_phase_2` |
| `Phase_3_P_Controller/run_phase_3.m` | Standalone launcher: Phase 3 | None | P-control sweep, transient disturbance | `phase_3_p_controller.m` | `run_phase_3` |
| `Phase_3_P_Controller/phase_3_p_controller.m` | Execution script: P-Controller | None | Kp sweep table, steady-state error analysis | `pitch_plant.m` | Executed by `run_phase_3` |
| `Phase_4_PI_Controller/run_phase_4.m` | Standalone launcher: Phase 4 | None | PI tuning, Pareto selection, freezes model | `phase_4_pi_controller.m` | `run_phase_4` |
| `Phase_4_PI_Controller/phase_4_pi_controller.m` | Execution script: PI Tuning & Freezing | None | Saves `frozen_pi_controller.mat`, CSVs | `get_robot_parameters.m`, `get_pitch_plant.m` | Executed by `run_phase_4` |
| `Phase_4_5_Disturbance_Frequency/run_phase_4_5.m`| Standalone launcher: Phase 4.5 | None | Disturbance rejection validation | `phase_4_5_disturbance.m` | `run_phase_4_5` |
| `Phase_4_5_Disturbance_Frequency/phase_4_5_disturbance.m`| Execution script: Disturbance Rejection | None | Validation status (PASS/FAIL), recovery times | `frozen_pi_controller.mat` | Executed by `run_phase_4_5` |
| `Phase_5_Hardware_Realistic/run_phase_5.m` | Standalone launcher: Phase 5 | None | Realistic simulation with noise & delay | `phase_5_hardware_realistic.m` | `run_phase_5` |
| `Phase_5_Hardware_Realistic/phase_5_hardware_realistic.m` | Execution script: Hardware Realism | None | 16 figures, performance comparison CSV | `frozen_pi_controller.mat` | Executed by `run_phase_5` |
| `Phase_4_6_Contact_Loss/run_phase_4_6.m` | Standalone launcher: Phase 4.6 | None | Evaluates leg contact loss faults | `phase_4_6_contact_loss.m` | `run_phase_4_6` |
| `Phase_4_6_Contact_Loss/phase_4_6_contact_loss.m` | Execution script: Fault Analysis | None | Fault summary CSV, duration sweep CSV | `frozen_pi_controller.mat` | Executed by `run_phase_4_6` |
| `Phase_4_7_Robustness/run_phase_4_7.m` | Standalone launcher: Phase 4.7 | None | Evaluates parametric plant uncertainty | `phase_4_7_robustness.m` | `run_phase_4_7` |
| `Phase_4_7_Robustness/phase_4_7_robustness.m` | Execution script: Robustness Analysis | None | 48-plant grid search, sensitivity rankings | `frozen_pi_controller.mat` | Executed by `run_phase_4_7` |
| **`Phase_8_Learning_Control_Benchmark/`** | | | | | |
| `Phase_8_.../run_phase_8.m` | Master Phase 8 Launcher | `'fast'` (optional string) | Full benchmark results struct | Controllers, training, simulation modules | `run_phase_8` or `run_phase_8('fast')` |
| `Phase_8_.../controllers/pi_baseline_controller.m` | PI Controller constructor | `cfg` struct | Controller instance struct | None | Constructor function |
| `Phase_8_.../controllers/pi_step.m` | Discrete-time step for PI | `ctrl, e, dt` | `[u, ctrl]` (anti-windup saturated) | None | Step function |
| `Phase_8_.../controllers/lstm_pitch_controller.m` | LSTM Controller constructor | `weights_or_mat, pi_cfg` | Controller instance struct | None | Constructor function |
| `Phase_8_.../controllers/lstm_step.m` | Recurrent forward step for LSTM | `ctrl, e, e_dot, theta, dt` | `[u, ctrl]` (recurrent inference) | None | Step function |
| `Phase_8_.../controllers/td3_pitch_controller.m` | TD3 Actor constructor | `actor_or_mat, pi_cfg` | Controller instance struct | None | Constructor function |
| `Phase_8_.../controllers/td3_step.m` | Deterministic Actor forward step | `ctrl, s` | `[u, ctrl]` (tanh saturated) | None | Step function |
| `Phase_8_.../training/generate_training_data.m` | Training trajectory generator | `n_trajectories, dt, T_horizon` | `data` struct with states/torques | `simulate_closed_loop.m` | Called by `run_phase_8` |
| `Phase_8_.../training/train_lstm_controller.m` | Supervised BPTT LSTM trainer | `data, hidden_dim, epochs` | Trained `weights` struct | None | Called by `run_phase_8` |
| `Phase_8_.../training/train_td3_controller.m` | Continuous Actor Policy optimizer | `max_episodes, u_sat` | Trained `actor` struct | None | Called by `run_phase_8` |
| `Phase_8_.../simulation/generate_test_scenarios.m`| Generates 4 disjoint test benchmarks | None | `scenarios` struct array | None | Called by `run_phase_8` |
| `Phase_8_.../simulation/simulate_closed_loop.m` | RK4 SISO closed-loop simulation | `ctrl_type, ctrl, scenario, dt` | `sim_out` history struct | Step functions | Called by `run_phase_8` |
| `Phase_8_.../simulation/compute_benchmark_metrics.m`| Standardized metric extractor | `sim_out, scenario` | Numerical `metrics` struct | None | Metric extraction function |
| `Phase_8_.../simulation/run_benchmark_suite.m` | Benchmark test suite runner | `controllers, test_suite` | Aggregated `benchmark_results` struct | `simulate_closed_loop.m` | Called by `run_phase_8` |
| `Phase_8_.../visualization/plot_comparative_responses.m`| Comparative response plotter | `benchmark_results, fig_dir` | 4 comparison figures saved to disk | None | Plotting function |
| `Phase_8_.../visualization/export_benchmark_tables.m`| CSV table and metrics exporter | `benchmark_results, tables_dir` | 5 CSV summary tables exported | None | Exporter function |
| **`04_COMMON/`** | | | | | |
| `04_COMMON/functions/step_metrics.m` | Extracts rise, settling, overshoot | `t, y, y_final` | `info` struct | None | Utility function |
| `04_COMMON/functions/format_poles.m` | Formats pole strings | `p` (pole vector) | Formatted string | None | Utility function |
| `04_COMMON/utilities/print_banner.m` | Displays console section banner | `title_str` | Console printout | None | Utility function |
| `04_COMMON/utilities/ensure_clean_env.m` | Ensures clean workspace safely | None | Safe workspace cleanup | None | Utility function |
| `04_COMMON/plotting/plot_step_response.m` | Standard step response plotter | `t, y, info, title_str` | Figure handle | None | Plotting function |
| `04_COMMON/plotting/plot_disturbance_response.m`| Standard disturbance plotter | `t, theta, u, T_d` | Figure handle | None | Plotting function |
| **`05_SIMULINK/`** | | | | | |
| `05_SIMULINK/scripts/simulink_init.m` | Workspace initializer for Simulink | None | Prepares base workspace variables | `robot_parameters.m`, `pitch_plant.m` | `simulink_init;` |
| **`Balance_Control/`** | | | | | |
| `Balance_Control/Master_Balance.m` | Balance subsystem entry launcher | `varargin` | Forwards to `run_project` | `Quadruped_Body_Pitch_Control` | `Master_Balance` |
| **`Gait_Movement/`** | | | | | |
| `Gait_Movement/Master_Gait.m` | Standalone gait foundation script | None | Saves `Master_Gait_Foundation.mat` | None | `Master_Gait` |
| `Gait_Movement/Phase_0_Interface/init_gait_state.m` | Standardized gait state constructor | Optional key-value parameters | `gaitState, interface` structs | None | Constructor function |
| `Gait_Movement/Phase_0_Interface/phase_1_interface.m` | Procedural gait interface script | None | Workspace interface verification | `init_gait_state.m` | `phase_1_interface;` |
| `Gait_Movement/Phase_2_Foot_Trajectory/phase_3_foot_trajectory.m` | Foot trajectory curve script | None | Single-leg foot curve in workspace | None | `phase_3_foot_trajectory;` |
| `Gait_Movement/Phase_3_Inverse_Kinematics/solve_leg_ik.m` | 3-DOF analytical leg IK solver | `x, y, z, L1, L2` | `[q1; q2; q3], feasible_flag` | None | Pure analytical function |
| `Gait_Movement/Phase_3_Inverse_Kinematics/phase_4_inverse_kinematics.m` | IK verification test script | None | Validates joint angles across workspace | `solve_leg_ik.m` | `phase_4_inverse_kinematics;` |
| `Gait_Movement/Phase_4_Single_Leg/run_phase_5_single_leg.m` | Single-leg trajectory launcher | None | Single-leg kinematics test | `Master_Gait.m` | `run_phase_5_single_leg;` |
| **`Robot_Movement/`** | | | | | |
| `Robot_Movement/Master_Robot.m` | Standalone robot dynamics foundation | None | Saves `Master_Robot_Foundation.mat` | None | `Master_Robot` |
| `Robot_Movement/Robot_Parameters/get_quadruped_geometry.m`| Authoritative robot geometry struct | None | Geometry struct ($L_1, L_2$, hips) | None | Pure struct accessor |
| `Robot_Movement/Robot_Parameters/phase_2_robot_geometry.m` | Geometry verification script | None | Console geometry verification | `get_quadruped_geometry.m` | `phase_2_robot_geometry;` |
| **`Integration/`** | | | | | |
| `Integration/Robot_State_Interface.m` | Telemetry to SISO error extractor | `robotState, reference_pitch` | `balance_input` struct ($e, \theta$) | None | Interface function |
| `Integration/Balance_Gait_Interface.m` | Pitch error to gait adaptation | `balanceState, robotState` | `gaitAdaptation` struct | None | Adaptation function |
| `Integration/Gait_Balance_Interface.m` | Combines gait angles & balance torque| `gaitState, balanceState` | `legCommand` struct ($q_{\text{command}}$) | None | Combiner function |
| `Integration/Disturbance_Interface.m` | Disturbance torque generator | `t, type` | `distState.tau_dist` | None | Disturbance function |
| `Integration/Master_Integration.m` | Closed-loop 4-subsystem co-simulation | None | Returns `results` struct | All interfaces | `results = Master_Integration();` |
| **`Gait_Optimization/`** | | | | | |
| `Gait_Optimization/get_dynamics_and_actuator_limits.m`| Motor, battery, & physical database | None | Central parameters struct | `get_quadruped_geometry.m` | Parameter accessor |
| `Gait_Optimization/calculate_gait_dynamics.m`| Analytical speed, torque, power calc | `step_length, freq, height, duty, params`| `dyn` dynamic struct | `get_dynamics_and_actuator_limits.m` | Dynamic calculation function |
| `Gait_Optimization/run_speed_safety_study.m`| Closed-loop study across 7 speeds | `candidates` (optional) | Summary table & candidate data | `calculate_gait_dynamics.m`, CPG | `study_results = run_speed_safety_study();` |
| `Gait_Optimization/plot_speed_optimization.m`| Comparative engineering plots | `study_results, save_dir` | 2 figures (actuators, dynamics) | None | Plotting function |
| `Gait_Optimization/run_gait_speed_optimization.m`| Master optimization launcher | None | Executes study & generates plots | `run_speed_safety_study.m` | `run_gait_speed_optimization` |
| **`Simulink_Gait/`** | | | | | |
| `Simulink_Gait/initialize_gait_model.m` | Central parameter loader for Simulink | None | Assigns `gait_cfg` in base workspace | `get_quadruped_geometry.m` | `gait_cfg = initialize_gait_model();` |
| `Simulink_Gait/build_gait_model.m` | Programmatic Simulink builder | `save_dir` (optional) | Generates `Quadruped_Gait_Unified.slx` | `initialize_gait_model.m`, Simulink | `build_gait_model();` |
| `Simulink_Gait/run_gait_simulation.m` | Discrete-time simulation engine | `t_stop` (optional) | `sim_results` struct (4.0s horizon) | `initialize_gait_model.m`, adapters | `sim_results = run_gait_simulation();` |
| `Simulink_Gait/validate_gait_model.m` | Comprehensive 11-test verification | None | `test_report` struct (Tests 1–11) | All Simulink_Gait adapters | `report = validate_gait_model();` |
| `Simulink_Gait/execute_gait_and_optimization_batch.m`| Standalone batch verification | None | Runs tests, optimization, & builder | `validate_gait_model.m`, optimization | `batch_out = execute_gait_and_optimization_batch();`|
| `Simulink_Gait/adapters/cpg_gait_generator.m` | 4-leg Trot phase oscillator | `t, frequency, offsets` | `phases, stance, swing` | None | Adapter function |
| `Simulink_Gait/adapters/four_leg_foot_trajectory.m` | Instantaneous 3D foot coordinate gen | `phases, L_stride, h_step, nominalFoot` | `foot_pos [4x3]` | None | Adapter function |
| `Simulink_Gait/adapters/four_leg_foot_trajectory_continuous.m`| Continuous-time foot coordinate gen | `phases, L_stride, h_step, nominalFoot` | `foot_pos [4x3]` | None | Adapter function |
| `Simulink_Gait/adapters/four_leg_ik_solver.m` | 12-DOF vector analytical IK wrapper | `foot_positions, L1, L2` | `q_gait [12x1], feasible` | `solve_leg_ik.m` | Adapter function |
| `Simulink_Gait/adapters/gait_balance_coupler.m` | Combines joint angles & pitch torque | `q_gait, u_pitch, k_torque` | `q_command [12x1]` | None | Adapter function |
| `Simulink_Gait/adapters/balance_adaptation_feedback.m`| Pitch-error step height modulator | `pitch_error` | `step_height_mod` | None | Adapter function |

---

## Section E — Exact MATLAB Execution Guide

### 1. Project Initialization
- **Purpose:** Configure MATLAB search paths dynamically to recognize all subsystems.
- **Working Directory:** `c:\Users\karti\Desktop\Projects\Subject\Quadrupled_Project`
- **Prerequisites:** None.
- **Command:**
  ```matlab
  project_setup
  ```
- **Expected Console Output:**
  ```text
  ============================================================
    QUADRUPED ROBOT BODY-PITCH CONTROL PROJECT
    Setup Initialized Successfully
  ============================================================
    Project Root : .../Quadrupled_Project
    Version      : 1.0.0
    Status       : All paths loaded into MATLAB environment.
  ============================================================
  ```
- **Verification:** Execute `which project_paths`. It should return `.../Quadruped_Body_Pitch_Control/01_SETUP/project_paths.m`.

---

### 2. Interactive Launcher (`run_project`)
- **Purpose:** Unified entry point to inspect and execute any phase interactively.
- **Command:**
  ```matlab
  run_project
  ```
- **Interactive Options:**
  - `[1]` Phase 1: Physical Model & Initial Condition Response
  - `[2]` Phase 2: SISO Open-Loop Plant Analysis
  - `[3]` Phase 3: Proportional (P) Feedback Controller Design
  - `[4]` Phase 4: PI Controller Design & Pareto Tuning
  - `[5]` Phase 4.5: Disturbance Rejection Validation
  - `[6]` Phase 5: Hardware-Realistic Simulation (Noise, Sampling, Saturation)
  - `[7]` Phase 6: Contact Loss / Leg Fault Analysis
  - `[8]` Phase 7: Robustness & Parametric Uncertainty Analysis
  - `[9]` Phase 8: Learning Control Benchmark (PI vs. LSTM vs. TD3)
  - `[10]` Master 1–3: Sequential Execution of Phases 1 to 3
  - `[11]` Master 4–4.5: Sequential Execution of Phases 4 to 4.5
  - `[A]` All: Executes all phases sequentially (1 to 8)
  - `[Q]` Quit launcher

---

### 3. Body-Pitch Control: Phase-by-Phase Direct Execution

#### Phase 1: Physical Model & Initial Response
- **Purpose:** Formulate the SISO rotational plant $G(s) = \frac{1}{J s^2 + B s + K}$ with $J=0.05, B=0.10, K=0.00$, and simulate initial condition recovery ($\theta_0 = 5^\circ$).
- **Command:**
  ```matlab
  run_phase_1
  ```
- **Outputs:** Step response figure and Test 3 initial condition state-space simulation plot.

#### Phase 2: SISO Open-Loop Analysis
- **Purpose:** Analyze open-loop poles ($s = 0, -2$), confirm Type 1 integrating behavior, and compute analytical impulse final value ($20/2 = 10.00\,\text{rad}$).
- **Command:**
  ```matlab
  run_phase_2
  ```
- **Outputs:** Pole-zero map and open-loop impulse/step response figures.

#### Phase 3: Proportional (P) Feedback Control
- **Purpose:** Sweep $K_p \in [0.05, 5.0]$, evaluate rise/settling time constraints ($M_p \le 25\%, t_s \le 4\,\text{s}, t_r \le 1\,\text{s}$), and analyze steady-state error under sustained disturbance torque ($e_{\text{ss}} = T_{d0}/K_p$).
- **Command:**
  ```matlab
  run_phase_3
  ```
- **Outputs:** Gain feasibility tables and disturbance comparison plots ($K_p = 0.25, 0.30$).

#### Phase 4: PI Controller Design, 2D Grid Sweep & Freezing
- **Purpose:** Sweep 2,500 $(K_p, K_i)$ combinations, apply Routh stability boundaries ($K_i < 2 K_p$), evaluate 3D Pareto dominance, select the optimal candidate ($K_p = 0.50, K_i = 0.20$), and freeze controller parameters.
- **Command:**
  ```matlab
  run_phase_4
  ```
- **Outputs:**
  - Saves `Quadruped_Body_Pitch_Control/07_DATA/processed/frozen_pi_controller.mat`.
  - Exports 3 CSV tables to `06_RESULTS/tables/` (`complete_sweep_results.csv`, `final_candidate_metrics.csv`, `p_vs_pi_comparison.csv`).
  - Generates 16 figures in `06_RESULTS/figures/`.

#### Phase 4.5: Disturbance Rejection Validation
- **Purpose:** Validate disturbance recovery against theoretical DC rejection ($G_d(0) = 0$) under transient ($0.10\,\text{N}\cdot\text{m}$ pulse) and sustained step disturbances.
- **Prerequisites:** Phase 4 frozen controller (loads automatically or computes fallback).
- **Command:**
  ```matlab
  run_phase_4_5
  ```
- **Outputs:** Sustained and transient recovery plots; console confirmation `PHASE 4.5 VALIDATION: PASS`.

#### Phase 5: Hardware-Realistic SISO Simulation
- **Purpose:** Evaluate the frozen PI controller under realistic embedded hardware non-idealities: IMU noise ($\sigma = 0.005\,\text{rad}$), 100 Hz sampling, 15 Hz low-pass filtering, actuator saturation ($\pm 0.40\,\text{N}\cdot\text{m}$), torque rate limiting ($5.0\,\text{N}\cdot\text{m/s}$), and anti-windup clamping.
- **Command:**
  ```matlab
  run_phase_5
  ```
- **Outputs:** Exports `06_RESULTS/tables/phase5_performance_comparison.csv` and 16 diagnostic figures.

#### Phase 6 (Phase 4.6): Contact-Loss & Leg Fault Analysis
- **Purpose:** Simulate temporary and persistent single-leg contact loss faults across all four legs (FL, FR, RL, RR) with a $25\%$ torque drop ($\alpha = 0.75$) and $T_d = \pm 0.25\,\text{N}\cdot\text{m}$ disturbance.
- **Command:**
  ```matlab
  run_phase_4_6
  % Alternative alias:
  run_phase_6
  ```
- **Outputs:** Exports 3 CSV tables (`phase6_fault_analysis_summary.csv`, severity sweep, duration sweep) and 16 figures.

#### Phase 7 (Phase 4.7): Parametric Robustness & Uncertainty Analysis
- **Purpose:** Perform a 48-plant grid search evaluating $\pm 30\%$ variation in body pitch inertia ($J \in [0.035, 0.065]$), damping ($B \in [0.07, 0.13]$), and parasitic stiffness ($K \in [0.0, 0.05]$).
- **Command:**
  ```matlab
  run_phase_4_7
  % Alternative alias:
  run_phase_7
  ```
- **Outputs:** Exports 8 CSV tables (including `final_project_metrics_summary.csv`) and 16 figures.

---

### 4. Phase 8: Learning-Control Benchmark Execution
- **Purpose:** Train and evaluate Supervised LSTM and Continuous Policy Actor against the frozen baseline PI controller across 4 disjoint test scenarios.
- **Command (Full Execution):**
  ```matlab
  run_phase_8
  ```
- **Command (Fast Verification Mode):**
  ```matlab
  run_phase_8('fast')
  ```
- **Expected Runtime:**
  - Full Mode: ~30 to 60 seconds (generates 20 training trajectories, trains LSTM for 35 epochs, trains TD3 Actor for 50 episodes, runs 4 benchmark scenarios, exports 5 CSV tables and 4 figures).
  - Fast Mode: ~10 to 15 seconds (reduced epochs for rapid pipeline verification).
- **Outputs:**
  - Models saved to `Quadruped_Body_Pitch_Control/03_PHASES/Phase_8_Learning_Control_Benchmark/controllers/`: `trained_lstm_model.mat`, `trained_td3_model.mat`.
  - CSV tables saved to `06_RESULTS/tables/`: `phase8_summary_metrics.csv`, `phase8_scenario1_metrics.csv` through `phase8_scenario4_metrics.csv`.
  - Figures saved to `06_RESULTS/figures/`: `phase8_fig01_scenario1_tracking.png` through `phase8_fig04_scenario4_uncertainty.png`.

---

### 5. Gait-Speed & Actuator Safety Optimization
- **Purpose:** Analyze 7 candidate forward walking speeds (0.064 m/s to 0.500 m/s), evaluate joint angular velocity, joint torque, mechanical power, and 7.4V battery current, and determine physical feasibility.
- **Command:**
  ```matlab
  run_gait_speed_optimization
  ```
- **Outputs:**
  - Exports CSV table: `Gait_Optimization/results/speed_safety_study_results.csv`.
  - Generates 2 engineering figures in `Gait_Optimization/results/`:
    - `speed_opt_fig01_constraints.png`: Speed vs. Servo velocity ($5.0\,\text{rad/s}$ limit), stall torque ($2.5\,\text{N}\cdot\text{m}$ limit), power demand, and pitch RMSE.
    - `speed_opt_fig02_time_responses.png`: Direct comparison between Baseline Trot (0.10 m/s) and Proposed Optimal Trot (0.21 m/s).

---

### 6. Simulink Unified Gait Verification & Simulation
- **Purpose:** Verify the unified quadruped gait architecture using the 11-test verification harness, and simulate closed-loop stepping.
- **Command (11-Test Verification Suite):**
  ```matlab
  validate_gait_model
  ```
- **Command (Discrete Co-Simulation Engine — No Simulink License Required):**
  ```matlab
  sim_results = run_gait_simulation(4.0);
  ```
- **Command (Programmatic Simulink Model Builder — Requires Simulink):**
  ```matlab
  build_gait_model
  ```
  *Generates:* `Simulink_Gait/Quadruped_Gait_Unified.slx`.
- **Command (Complete Batch Suite):**
  ```matlab
  execute_gait_and_optimization_batch
  ```

---

### 7. Full Subsystem Co-Integration Test
- **Purpose:** Validate two-way closed-loop coupling across all 4 project subsystems: Gait Movement $\leftrightarrow$ Robot Telemetry $\leftrightarrow$ Balance Control $\leftrightarrow$ Gait Adaptation.
- **Command:**
  ```matlab
  results = Master_Integration();
  ```
- **Expected Console Output:**
  ```text
  ============================================================
    QUADRUPED INTEGRATION: TWO-WAY CLOSED-LOOP VERIFICATION
  ============================================================
  Loaded Frozen PI Controller: Kp = 0.50, Ki = 0.20
  Running closed-loop simulation horizon [0, 4.0s]...
  >>> INTEGRATION SUCCESSFUL: Closed-loop data flow verified across all 4 subsystems.
  ```

---

## Section F — Phase 8: PI vs. LSTM vs. TD3

### 1. Conceptual & Mathematical Formulation of the Three Controllers

```
┌─────────────────────────────────────────────────────────────────────────────────────────────────┐
│                                 CONTROLLER ARCHITECTURE COMPARISON                              │
├───────────────────────────────┬─────────────────────────────────┬───────────────────────────────┤
│    1. FROZEN BASELINE PI      │     2. RECURRENT LSTM NETWORK   │  3. CONTINUOUS TD3 / ACTOR   │
├───────────────────────────────┼─────────────────────────────────┼───────────────────────────────┤
│ - Classical Linear Feedback   │ - Supervised Deep Learning      │ - Direct Continuous Policy    │
│ - Integral Tracking Action    │ - Recurrent Hidden State Memory │ - 2-Layer MLP (24x24 ReLU)    │
│ - Actuator Anti-Windup Clamp  │ - Multi-step Error Sequences    │ - Scaled tanh Output Saturation│
│ - Zero Inference Latency      │ - Dynamic Memory of Disturbances│ - High Dynamic Responsiveness │
└───────────────────────────────┴─────────────────────────────────┴───────────────────────────────┘
```

1. **Frozen Conventional PI Controller (Baseline):**
   - Equation:
     $$u(t) = \text{sat}\left( K_p e(t) + K_i \int_0^t e(\tau)\,d\tau \right)$$
   - Configured Parameters: $K_p = 0.50\,\text{N}\cdot\text{m/rad}$, $K_i = 0.20\,\text{N}\cdot\text{m/rad}\cdot\text{s}$.
   - Actuator Torque Saturation: $u_{\text{sat}} = \pm 0.40\,\text{N}\cdot\text{m}$.
   - Anti-Windup Clamping: Integrator accumulation halts whenever the actuator saturates and the sign of the error matches the saturated control action.
2. **LSTM Recurrent Neural Network Controller:**
   - Architecture: 3-dimensional input vector $\mathbf{x}_t = [e(t), \dot{e}(t), \theta(t)]^T$, 16 hidden LSTM units, and 1 continuous output torque.
   - Gating Formulation:
     $$\mathbf{f}_t = \sigma(\mathbf{W}_f \mathbf{x}_t + \mathbf{U}_f \mathbf{h}_{t-1} + \mathbf{b}_f) \quad \text{(Forget Gate)}$$
     $$\mathbf{i}_t = \sigma(\mathbf{W}_i \mathbf{x}_t + \mathbf{U}_i \mathbf{h}_{t-1} + \mathbf{b}_i) \quad \text{(Input Gate)}$$
     $$\tilde{\mathbf{c}}_t = \tanh(\mathbf{W}_c \mathbf{x}_t + \mathbf{U}_c \mathbf{h}_{t-1} + \mathbf{b}_c) \quad \text{(Candidate Cell)}$$
     $$\mathbf{c}_t = \mathbf{f}_t \odot \mathbf{c}_{t-1} + \mathbf{i}_t \odot \tilde{\mathbf{c}}_t \quad \text{(Cell State Memory)}$$
     $$\mathbf{o}_t = \sigma(\mathbf{W}_o \mathbf{x}_t + \mathbf{U}_o \mathbf{h}_{t-1} + \mathbf{b}_o) \quad \text{(Output Gate)}$$
     $$\mathbf{h}_t = \mathbf{o}_t \odot \tanh(\mathbf{c}_t) \quad \text{(Hidden State Output)}$$
     $$u(t) = \text{sat}(\mathbf{W}_y \mathbf{h}_t + b_y)$$
   - Supervised Training: Trained using Backpropagation Through Time (BPTT) and Adam optimization on multi-step trajectory datasets generated from randomized reference profiles.
3. **TD3 / Continuous Policy Controller:**
   - Architecture: Input state vector $\mathbf{s}_t = [e(t), \dot{e}(t), \theta(t), \int e\,dt]^T \in \mathbb{R}^4$.
   - Feedforward Policy: Layer 1 ($24$ neurons, ReLU) $\to$ Layer 2 ($24$ neurons, ReLU) $\to$ Bounded $\tanh$ output layer scaled by $u_{\text{sat}} = 0.40\,\text{N}\cdot\text{m}$.
   - Custom Optimization Mechanism (Technical Reality Audit):
     > **Important Engineering Audit Note:**  
     > The file `train_td3_controller.m` implements a **Direct Policy Optimization / Augmented Random Search (ARS)** algorithm across continuous episodic rollouts rather than an off-policy replay-buffer Actor-Twin-Critic update (as the RL Toolbox is not required). It perturbs the actor weights across $P=8$ candidates per episode, computes a quadratic tracking reward $r_t = -(15 e^2 + 0.2 \dot{e}^2 + 0.1 (u/u_{\text{sat}})^2)$, normalizes candidate rewards, and applies policy gradient steps. The forward inference module (`td3_step.m`) executes the deterministic Actor policy network.

### 2. Disjoint Training and Benchmark Test Scenarios
To guarantee unbiased scientific evaluation, the training dataset and benchmark test scenarios are strictly decoupled:
- **Training Distribution:** 20 randomized reference profiles with stochastic step transitions between $[-0.10, +0.10]\,\text{rad}$ ($[-5.7^\circ, +5.7^\circ]$), random transient pulses ($|T_d| \le 0.10\,\text{N}\cdot\text{m}$), and small uniform variations in inertia $J \in [0.045, 0.055]$ and damping $B \in [0.09, 0.11]$.
- **Benchmark Test Suite (Never seen during training):**
  1. **Test 1: Dynamic Trajectory Tracking:** Step transition from $0\,\text{rad} \to +5^\circ (+0.0873\,\text{rad})$ at $t=1\,\text{s}$, to $-3^\circ (-0.0524\,\text{rad})$ at $t=3\,\text{s}$, followed by a $0.5\,\text{Hz}$ continuous sinusoid. Disturbance torque $T_d = 0$.
  2. **Test 2: Sustained Torque Disturbance Rejection:** Regulation at $\theta_d = 0\,\text{rad}$ with a sustained external torque impact $T_d = +0.15\,\text{N}\cdot\text{m}$ applied from $t=2.0\,\text{s}$ to $4.5\,\text{s}$.
  3. **Test 3: Contact Loss / Foot Slip Fault:** Regulation at $\theta_d = 0\,\text{rad}$ under a sudden severe torque impact $T_d = -0.25\,\text{N}\cdot\text{m}$ ($t=2.0\,\text{s}$ to $3.5\,\text{s}$) with a simultaneous $30\%$ drop in ground damping ($B = 0.070$).
  4. **Test 4: Plant Uncertainty & Parameter Mismatch:** Alternating $\pm 4^\circ$ square steps with physical plant parameter variations: inertia $J$ increased by $+35\%$ ($J = 0.0675\,\text{kg}\cdot\text{m}^2$) and damping $B$ reduced by $-30\%$ ($B = 0.070\,\text{N}\cdot\text{m}\cdot\text{s/rad}$).

### 3. Quantitative Performance Metrics
The benchmark suite evaluates each controller using 8 standardized criteria:
1. **RMSE (Root Mean Square Error):** $\text{RMSE} = \sqrt{\frac{1}{N}\sum_{k=1}^N (e_k)^2}$ [rad].
2. **IAE (Integrated Absolute Error):** $\text{IAE} = \sum_{k=1}^N |e_k| \Delta t$ [$\text{rad}\cdot\text{s}$].
3. **Peak Pitch Deviation:** $\max_k |\theta_k|$ [rad and deg].
4. **Overshoot ($M_p$):** Percentage by which peak response exceeds commanded step target.
5. **Settling Time ($t_s$):** Time required to enter and remain within a $\pm 5\%$ error boundary.
6. **Recovery Time ($t_{\text{rec}}$):** Time required to re-enter $\pm 0.01\,\text{rad}$ error band after disturbance release.
7. **Control Energy ($E_u$):** $\int_0^T u(t)^2\,dt \approx \sum_{k=1}^N (u_k)^2 \Delta t$ [$\text{N}^2\cdot\text{m}^2\cdot\text{s}$].
8. **Inference Time:** Average CPU computational execution time per time step ($\Delta t = 0.005\,\text{s}$) [microseconds].

---

## Section G — Gait-Speed & Actuator Safety Optimization

### 1. Fundamental Walking Speed Formulation
The theoretical forward locomotion velocity of a trotting quadruped is governed by:
$$v = L_{\text{stride}} \cdot f_{\text{gait}}$$
where:
- $L_{\text{stride}}$ is the stride length [m] (distance covered per complete gait cycle).
- $f_{\text{gait}}$ is the gait frequency / cadence [Hz] ($f_{\text{gait}} = 1/T_{\text{gait}}$).

#### Crucial Engineering Distinction: Theoretical vs. Physical Speed
- **Theoretical Speed ($L_{\text{stride}} \cdot f_{\text{gait}}$):** Assumes rigid contact with zero foot slip ($\mu \ge \mu_{\text{req}}$), instantaneous leg swing, and zero actuator torque/velocity saturation.
- **Physical Feasible Speed:** Constrained by:
  1. Maximum servo no-load speed $\omega_{\text{max}}$.
  2. Peak joint torque $\tau_{\text{stall}}$ and thermal continuous rating $\tau_{\text{rated}}$.
  3. Peak electrical battery discharge current $I_{\text{bus}}$.
  4. Dynamic ground traction limits (avoiding foot slip within the friction cone: $|F_x/F_z| \le \mu$).
  5. Pitch stability margin (preventing body tumble when foot touchdown impacts excite the pitch plant).

### 2. Multi-Candidate Speed Study
The script `run_speed_safety_study.m` systematically evaluates 7 candidate gaits:

| Candidate ID | Label | Stride $L$ [m] | Freq $f$ [Hz] | Height $h$ [m] | Speed $v$ [m/s] | Max $\omega$ [rad/s] | Max $\tau$ [N.m] | Avg Power [W] | Avg Current [A] | Status & Reason |
| :--- | :--- | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :--- |
| **C1** | Crawl / Slow | 0.08 | 0.80 | 0.035 | **0.064** | 1.84 | 0.98 | 3.12 | 0.52 | **PASS (FEASIBLE)** |
| **C2** | Baseline Trot | 0.10 | 1.00 | 0.040 | **0.100** | 2.45 | 1.15 | 4.85 | 0.78 | **PASS (FEASIBLE)** |
| **C3** | Moderate Fast | 0.12 | 1.25 | 0.038 | **0.150** | 3.28 | 1.34 | 7.42 | 1.18 | **PASS (FEASIBLE)** |
| **C4** | Proposed Optimal | 0.14 | 1.50 | 0.035 | **0.210** | 4.22 | 1.58 | 10.95 | 1.72 | **PASS (FEASIBLE)** |
| **C5** | High Speed | 0.16 | 1.75 | 0.032 | **0.280** | 5.26 | 1.86 | 15.60 | 2.45 | **FAIL (Omega_Limit)** |
| **C6** | Extreme Speed | 0.18 | 2.00 | 0.030 | **0.360** | 6.42 | 2.18 | 21.80 | 3.40 | **FAIL (Omega_Limit)** |
| **C7** | Boundary Limit | 0.20 | 2.50 | 0.028 | **0.500** | 8.85 | 2.82 | 38.50 | 5.95 | **FAIL (Omega, Torque, Stability)** |

### 3. Classification of Physical Parameters: Facts vs. Assumptions
In `get_dynamics_and_actuator_limits.m`, physical values are explicitly tagged:
- **Datasheet / Authoritative Spec `[M]`:** Gravity $g = 9.81\,\text{m/s}^2$, nominal battery voltage $V_{\text{nom}} = 7.4\,\text{V}$ (2S LiPo), max pack current $30.0\,\text{A}$.
- **Experimentally Verified / Code Constant `[E]`:** Link lengths $L_1 = 0.12\,\text{m}$, $L_2 = 0.15\,\text{m}$, body pitch inertia $J = 0.05\,\text{kg}\cdot\text{m}^2$, damping $B = 0.10\,\text{N}\cdot\text{m}\cdot\text{s/rad}$, standing clearance $z_0 = -0.22\,\text{m}$.
- **Simulation Assumptions `[S]` (Require physical benchtop measurement):**
  - Servo no-load speed $\omega_{\text{max}} = 5.0\,\text{rad/s}$ (RDS3225 / STS3215 class).
  - Stall torque $\tau_{\text{stall}} = 2.5\,\text{N}\cdot\text{m}$, thermal rated torque $\tau_{\text{rated}} = 0.80\,\text{N}\cdot\text{m}$.
  - Indoor tile/wood friction coefficient $\mu = 0.60$.
  - Total robot mass $M = 5.0\,\text{kg}$.
  - Neglected link inertia, gear backlash, motor armature resistance, and dynamic battery voltage sag.

---

## Section H — Simulink Guide

### 1. Model Files & Status Inspection
- **Architectural Specification (`Simulink/`):** Contains `Simulink/README.md` defining the conceptual multi-subsystem architecture (`Quadruped_Main.slx`).
- **Unified Simulink Implementation (`Simulink_Gait/`):** Contains programmatic builder `build_gait_model.m` which compiles `Quadruped_Gait_Unified.slx` on systems with a Simulink license.
- **Pitch Control Simulink Support (`Quadruped_Body_Pitch_Control/05_SIMULINK/`):** Contains `simulink_init.m` for preparing the base workspace for Simulink model callbacks or InitFcn.

### 2. Workspace Initialization Script
Before opening or simulating any Simulink model, execute:
```matlab
% For Body-Pitch Control Simulink models:
simulink_init

% For Unified Gait Simulink models:
gait_cfg = initialize_gait_model();
```
*What `initialize_gait_model.m` does:* Loads link lengths ($L_1, L_2$), nominal foot roots, trot phase offsets, frozen PI gains ($K_p=0.50, K_i=0.20$), and plant parameters into variable `gait_cfg` in the base workspace.

### 3. Programmatic Model Generation
To compile and generate the Simulink block harness programmatically:
```matlab
build_gait_model
```
*What this command does:*
- Verifies base workspace parameters.
- Invokes `new_system('Quadruped_Gait_Unified')`.
- Configures a Fixed-Step Discrete solver with fundamental sample time $\Delta t = 0.005\,\text{s}$ (200 Hz).
- Constructs 7 interconnected subsystems: Gait Configuration, CPG Phase Oscillator, 3D Foot Trajectory, 12-DOF Inverse Kinematics, Gait-Balance Coupler, Robot Movement Telemetry, and Disturbance Pulse Generator.
- Saves `Quadruped_Gait_Unified.slx` in `Simulink_Gait/`.

### 4. Running Simulation Without a Simulink License
If a Simulink license is unavailable, the repository includes a full-fidelity native MATLAB discrete-time simulation engine:
```matlab
sim_results = run_gait_simulation(4.0);
```
This executes the exact discrete state-machine equations at 200 Hz and returns full 12-DOF joint commands, body pitch trajectory, and disturbance response structs.

---

## Section I — Results and Output Files

All generated artifacts are organized into structured directories:

### 1. Numerical CSV Tables (`Quadruped_Body_Pitch_Control/06_RESULTS/tables/`)
- `complete_sweep_results.csv`: 2,500 candidate PI gain evaluations from Phase 4.
- `final_candidate_metrics.csv`: Selected optimal PI gains ($K_p=0.50, K_i=0.20$).
- `p_vs_pi_comparison.csv`: Time-domain performance metrics comparing P vs. PI controllers.
- `phase5_performance_comparison.csv`: Phase 5 hardware realism metrics (noise, delay, saturation).
- `phase6_fault_analysis_summary.csv`: Single-leg contact loss fault recovery data.
- `phase7_parameter_sensitivity_summary.csv`: 48-plant parameter uncertainty analysis.
- `final_project_metrics_summary.csv`: Overall summary of project performance metrics.
- `phase8_summary_metrics.csv`: Phase 8 controller benchmark comparison table (PI vs. LSTM vs. TD3).

### 2. Exported Workspace `.mat` Files
- `Quadruped_Body_Pitch_Control/07_DATA/processed/frozen_pi_controller.mat`: Frozen controller gains (`Kp_final`, `Ki_final`, `C_final`, `T_final`, `info_final`).
- `Quadruped_Body_Pitch_Control/06_RESULTS/exported/phase8_benchmark_results.mat`: Full simulation trajectories and metrics structs for Phase 8.
- `Gait_Movement/Master_Gait_Foundation.mat`: Standalone gait trajectory dataset.
- `Robot_Movement/Master_Robot_Foundation.mat`: Standalone multibody telemetry dataset.

### 3. Engineering Figures (`Quadruped_Body_Pitch_Control/06_RESULTS/figures/`)
- Figures `phase4_fig01` through `phase4_fig16`: Phase 4 Pareto trade-offs and Bode margins.
- Figures `phase5_fig01` through `phase5_fig16`: Phase 5 sensor noise, IMU delay, and saturation diagnostics.
- Figures `phase6_fig01` through `phase6_fig16`: Phase 6 contact loss fault responses.
- Figures `phase7_fig01` through `phase7_fig16`: Phase 7 parameter uncertainty sweeps.
- Figures `phase8_fig01` through `phase8_fig04`: Phase 8 multi-controller comparative responses.

### 4. Overwrite Protection & Data Safety
- Re-running a phase will regenerate its respective figures and CSV tables in `06_RESULTS/`.
- If preserving previous experimental runs is required, create a timestamped backup copy:
  ```matlab
  copyfile('Quadruped_Body_Pitch_Control/06_RESULTS', 'Quadruped_Body_Pitch_Control/06_RESULTS_BACKUP');
  ```

---

## Section J — Practical Troubleshooting Guide

### 1. `Undefined function or variable '...'`
- **Cause:** MATLAB search paths were reset or `project_setup` was not executed.
- **Diagnostic:**
  ```matlab
  which project_paths
  ```
- **Fix:** Execute `project_setup` from the project root.

---

### 2. `Foot position outside IK workspace`
- **Cause:** Commanded foot step length or height exceeds the kinematic leg reach ($L_1 + L_2 = 0.12 + 0.15 = 0.27\,\text{m}$).
- **Diagnostic:**
  ```matlab
  D = sqrt(x^2 + y^2 + z^2);
  fprintf('Commanded extension: %.3f m (Max reach: 0.270 m)\n', D);
  ```
- **Fix:** Ensure $L_{\text{stride}} \le 0.18\,\text{m}$ and standing height clearance $z_0 \ge -0.25\,\text{m}$.

---

### 3. `license('test', 'Control_Toolbox') returns 0`
- **Cause:** Control System Toolbox is missing or unlicensed.
- **Diagnostic:**
  ```matlab
  ver('control')
  ```
- **Fix:** Install the Control System Toolbox via the MATLAB Add-On Explorer.

---

### 4. Phase 8 Training Takes Too Long
- **Cause:** Full training mode generates 20 trajectories and trains LSTM for 35 epochs and TD3 for 50 episodes.
- **Diagnostic:** Check command arguments.
- **Fix:** Run the fast verification mode:
  ```matlab
  run_phase_8('fast')
  ```

---

### 5. `Simulink:Engine:LicenseFailed` or Missing `.slx`
- **Cause:** Simulink is not installed or unlicensed.
- **Fix:** Run the native discrete simulation engine:
  ```matlab
  sim_results = run_gait_simulation(4.0);
  ```
  It executes the identical mathematical model without requiring Simulink.

---

## Section K — Recommended Run Order

For a student starting with a clean MATLAB session, execute the project in this sequence:

### Stage 1: Environment Verification & Setup
1. Launch MATLAB and set working directory to `Quadrupled_Project`.
2. Run environment initialization:
   ```matlab
   project_setup
   ```
3. Verify plant dynamics:
   ```matlab
   validate_plant
   ```

### Stage 2: Classical Pitch Control Verification
4. Run Phase 1 (Physical model & initial state response):
   ```matlab
   run_phase_1
   ```
5. Run Phase 2 (SISO open-loop poles/zeros):
   ```matlab
   run_phase_2
   ```
6. Run Phase 3 (P-controller sweep):
   ```matlab
   run_phase_3
   ```
7. Run Phase 4 (PI tuning, 3D Pareto analysis, parameter freezing):
   ```matlab
   run_phase_4
   ```
8. Run Phase 4.5 (Disturbance rejection validation):
   ```matlab
   run_phase_4_5
   ```

### Stage 3: Hardware Realism & Robustness
9. Run Phase 5 (Noise, IMU sampling, saturation):
   ```matlab
   run_phase_5
   ```
10. Run Phase 6 (Leg contact loss fault analysis):
    ```matlab
    run_phase_4_6
    ```
11. Run Phase 7 (Parametric uncertainty analysis):
    ```matlab
    run_phase_4_7
    ```

### Stage 4: Advanced Extensions
12. Run Phase 8 Learning Benchmark (Fast Mode):
    ```matlab
    run_phase_8('fast')
    ```
13. Run Gait-Speed Safety Optimization:
    ```matlab
    run_gait_speed_optimization
    ```
14. Run Unified Gait 11-Test Verification Suite:
    ```matlab
    validate_gait_model
    ```
15. Run Closed-Loop 4-Subsystem Co-Simulation:
    ```matlab
    results = Master_Integration();
    ```

---

## Section L — Practical Verification Checklist

Follow this checklist to verify your repository setup and simulation results:

- [x] **Project paths verified:** Dynamic discovery active via `project_paths.m`.
- [x] **MATLAB environment verified:** Control System Toolbox detected.
- [x] **Physical baseline verified:** Parameters confirmed ($J = 0.05\,\text{kg}\cdot\text{m}^2, B = 0.10\,\text{N}\cdot\text{m}\cdot\text{s/rad}, K = 0.00\,\text{N}\cdot\text{m/rad}$).
- [x] **Classical control phases executed:** Phases 1, 2, 3, 4, 4.5 verified.
- [x] **Controller frozen:** Baseline gains set to $K_p = 0.50, K_i = 0.20$ with anti-windup clamping.
- [x] **Advanced robustness tested:** Phases 5 (Hardware Realism), 4.6 (Faults), and 4.7 (Robustness) verified.
- [x] **Phase 8 ML benchmark executed:** Custom native LSTM and TD3 continuous Actor verified against baseline PI.
- [x] **Gait-speed optimization verified:** 7 candidate gaits evaluated; C1–C4 pass, C5–C7 fail on motor limits.
- [x] **Simulink architecture inspected:** Programmatic builder and native simulation engine verified.
- [x] **Integration verified:** Closed-loop data flow verified across Gait, Robot, Balance, and Adaptation.
- [x] **Physical hardware assumptions documented:** Motor, battery, and contact assumptions classified.
