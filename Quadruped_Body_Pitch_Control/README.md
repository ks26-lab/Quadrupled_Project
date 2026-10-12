# Quadruped Robot Body-Pitch Control Project

## 1. Project Purpose
This project implements the modeling, analysis, and feedback control system for the body-pitch degree of freedom of a quadrupedal robot. The system controls the actual body pitch angle ($\theta$) in response to desired pitch angle commands ($\theta_d$) and provides rejection of external pitch torque disturbances ($T_d$) caused by foot contact impacts and terrain slope variations.

The control pipeline progresses through systematic development phases:
- **Phase 1:** Physical parameter specification, SISO transfer function, and initial condition response (state-space).
- **Phase 2:** SISO open-loop plant analysis (poles/zeros, step/impulse responses, analytical validation).
- **Phase 3:** Proportional ($P$) feedback control, systematic gain sweep ($K_p \in [0.05, 5.0]$), feasible constraint filtering, baseline comparison, and torque disturbance rejection.
- **Phase 4:** PI feedback controller design, 2D gain parameter sweep ($K_p, K_i \in [0.01, 0.50]$), 3D Pareto performance trade-off analysis, overshoot-priority candidate selection, and parameter freezing.
- **Phase 4.5:** Closed-loop disturbance rejection validation under sustained and transient pitch torque disturbances with automatic recovery time analysis and theoretical DC validation.
- **Roadmap Phases (4.6, 4.7, 5):** Contact loss during gait transitions, parametric robustness margins, and hardware-realistic realization (saturation, delay, discrete execution).

---

## 2. Directory Structure

```
Quadruped_Body_Pitch_Control/
│
├── 00_MAIN/
│   ├── run_project.m                # Central interactive & programmatic launcher
│   ├── main_phase1_to_3.m           # Frozen master script for Phases 1-3
│   ├── main_phase4_to_4_5.m         # Frozen master script for Phases 4-4.5
│   ├── main.m                       # Backward compatibility alias
│   └── PI_Controller.m              # Backward compatibility alias
│
├── 01_SETUP/
│   ├── project_paths.m              # Dynamic path manager (no hardcoded absolute paths)
│   ├── project_setup.m              # Setup verifier & path initializer
│   └── project_config.m             # Non-control project metadata & tolerances
│
├── 02_MODEL/
│   ├── dynamics/
│   │   ├── pitch_plant.m            # SISO plant G(s) and state-space sys_ss
│   │   └── get_pitch_plant.m        # Functional plant accessor
│   ├── parameters/
│   │   ├── robot_parameters.m       # Authoritative physical parameters (J, B, K)
│   │   └── get_robot_parameters.m   # Functional parameter accessor
│   └── validation/
│       └── validate_plant.m         # Analytical plant validation
│
├── 03_PHASES/
│   ├── Phase_1_Model/
│   │   ├── run_phase_1.m            # Standalone launcher
│   │   └── phase_1_model.m          # Phase 1 script
│   ├── Phase_2_SISO/
│   │   ├── run_phase_2.m            # Standalone launcher
│   │   └── phase_2_siso.m           # Phase 2 script
│   ├── Phase_3_P_Controller/
│   │   ├── run_phase_3.m            # Standalone launcher
│   │   └── phase_3_p_controller.m   # Phase 3 script
│   ├── Phase_4_PI_Controller/
│   │   ├── run_phase_4.m            # Standalone launcher
│   │   └── phase_4_pi_controller.m  # Phase 4 script
│   ├── Phase_4_5_Disturbance_Frequency/
│   │   ├── run_phase_4_5.m          # Standalone launcher
│   │   └── phase_4_5_disturbance.m # Phase 4.5 script
│   ├── Phase_4_6_Contact_Loss/      # Roadmap Phase 4.6 (Launcher + README)
│   ├── Phase_4_7_Robustness/        # Roadmap Phase 4.7 (Launcher + README)
│   └── Phase_5_Hardware_Realistic/  # Roadmap Phase 5 (Launcher + README)
│
├── 04_COMMON/
│   ├── functions/                   # Reusable analysis functions (step_metrics, format_poles)
│   ├── utilities/                   # Environment utilities (print_banner, ensure_clean_env)
│   └── plotting/                    # Standardized plotting utilities
│
├── 05_SIMULINK/
│   ├── models/                      # Simulink .slx / .mdl models
│   ├── scripts/                     # Pre-simulation setup scripts (simulink_init.m)
│   └── data/                        # Simulation buses and signal mat-files
│
├── 06_RESULTS/
│   ├── figures/                     # Output figures (.fig, .png)
│   ├── tables/                      # Output metric tables (.csv, .mat)
│   └── exported/                    # Exported logs and reports
│
├── 07_DATA/
│   ├── input/                       # External test sequences and reference trajectories
│   └── processed/                   # Frozen controller state (frozen_pi_controller.mat)
│
├── 08_DOCUMENTATION/
│   ├── dependency_map.md            # Detailed phase dependency map
│   ├── project_structure_report.md  # Audit and reorganization report
│   └── QUADRUPED-ROBOT-GAIT-and-BALANCE-CONTROL (1).pdf
│
└── 09_ARCHIVE/
    ├── pre_reorganization/          # Exact pre-reorganization frozen backups
    └── autosaves/                   # Editor autosave history (.asv)
```

---

## 3. How to Initialize MATLAB

Start MATLAB and navigate to either the repository root or the project folder. Run:
```matlab
project_setup
```
This performs the following actions:
1. Dynamically detects the project root folder without any hardcoded Windows drive letters or user usernames.
2. Adds all required project folders to the MATLAB search path.
3. Validates folder integrity and confirms setup in the command window.

---

## 4. How to Run the Entire Project

To run through the project or launch an interactive execution menu:
```matlab
run_project
```
You can also run all phases in sequence programmatically:
```matlab
run_project('all')
```
Or execute the frozen sequential master scripts:
```matlab
main_phase1_to_3      % Runs original master script for Phases 1-3
main_phase4_to_4_5    % Runs original master script for Phases 4-4.5
```

---

## 5. How to Run an Individual Phase

Each phase has a dedicated launcher that can be executed from **any current MATLAB working directory**:

```matlab
run_phase_1      % Physical Model & Initial Condition Response
run_phase_2      % SISO Plant & Open-Loop Analysis
run_phase_3      % Proportional Feedback Controller Design & Analysis
run_phase_4      % PI Controller Design, Pareto Front & Candidate Selection
run_phase_4_5    % Disturbance Rejection Validation & Recovery Analysis
run_phase_5      % Hardware-Realistic SISO Simulation (Sensor Noise, Sampling, Saturation)
run_phase_6      % One-Leg Contact-Loss / Fault Analysis (run_phase_4_6)
run_phase_7      % Robustness, Model Uncertainty & Final SISO Validation (run_phase_4_7)
```

You can also call the main launcher with the phase ID:
```matlab
run_project('7')
```

---

## 6. Where Model Parameters Live

There is strictly **one authoritative source** for physical model parameters:
- **Location:** `02_MODEL/parameters/robot_parameters.m`
- **Values:**
  - $J = 0.05\text{ kg}\cdot\text{m}^2$ (Body pitch moment of inertia)
  - $B = 0.10\text{ N}\cdot\text{m}\cdot\text{s/rad}$ (Pitch damping coefficient)
  - $K = 0.00\text{ N}\cdot\text{m/rad}$ (Initial restoring stiffness)

No controller gains or physical parameters are duplicated elsewhere.

---

## 7. Where Results and Exported Outputs Live

- **Saved Figures:** `06_RESULTS/figures/`
- **Exported Tables:** `06_RESULTS/tables/`
- **Processed Datasets & Controller Candidates:** `07_DATA/processed/`
- **Simulation Logs & Reports:** `06_RESULTS/exported/`

---

## 8. Where Simulink Models Live

- **Models:** Place `.slx` and `.mdl` models in `05_SIMULINK/models/`.
- **Pre-simulation Setup:** Run `simulink_init` before simulating any pitch model to ensure $J, B, K, G(s), \text{sys\_ss}, K_p, K_i$ are initialized in the base workspace.

---

## 9. How Dependencies Are Handled

- **Dynamic Plant Initialization:** If an isolated phase launcher (such as `run_phase_2` or `run_phase_3`) is run in a fresh MATLAB session, it automatically ensures $G(s)$ is initialized via `pitch_plant` rather than failing with `Undefined variable G`.
- **Decoupled Controller Loading:** When running Phase 4.5 in isolation (`run_phase_4_5`), the launcher looks for the frozen controller parameters in `07_DATA/processed/frozen_pi_controller.mat` or executes Phase 4 candidate selection automatically if unpopulated.
- **No Base-Workspace Fragility:** Individual phase scripts do not issue `clear all` or `clc`, ensuring that base workspace dependencies and test harnesses remain intact.

---

## 10. How to Add a New Phase Without Breaking the Project

To add a future phase (e.g. `Phase_4_6_Contact_Loss`):
1. Create or edit the script in `03_PHASES/Phase_4_6_Contact_Loss/phase_4_6_contact.m`.
2. Ensure the script begins with dependency resolution:
   ```matlab
   if ~exist('G', 'var'), pitch_plant; end
   if ~exist('C_final', 'var'), run_phase_4; end
   ```
3. Use the standardized launcher pattern in `03_PHASES/Phase_4_6_Contact_Loss/run_phase_4_6.m`:
   ```matlab
   this_dir = fileparts(mfilename('fullpath'));
   project_root = fileparts(fileparts(this_dir));
   addpath(fullfile(project_root, '01_SETUP'));
   project_setup(true);
   phase_4_6_contact;
   ```
4. Register the phase in `01_SETUP/project_config.m` and `00_MAIN/run_project.m`.
