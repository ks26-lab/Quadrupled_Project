# Quadruped Robot Gait and Balance Control: Modular Architecture

## Executive Overview
This repository implements a modular, high-performance architecture for **Quadruped Robot Gait and Balance Control**, cleanly separating computational algorithms, Simulink models, shared configuration, real-time 3D simulation, and automated test harnesses.

---

## 1. Quickstart Guide (From Clean MATLAB Command Prompt)

```matlab
% 1. Navigate to project root
cd('c:\Users\karti\Desktop\Projects\Subject\Quadrupled_Project');

% 2. Build and validate project dependencies
build_project();

% 3. Run automated unit & regression test suite
run_tests();

% 4. Run an individual movement phase in isolation (Phases 1 to 8)
run_phase(4); % Executes Phase 4 (12-DOF Inverse Kinematics)

% 5. Launch the Real-Time 3D Simulation & Conveyor Dashboard
run_project('3d');

% 6. Run Simulink Benchmark Models
run_project('pid');      % Model A: Baseline Discrete PI/PID
run_project('learning'); % Model B: Supervised LSTM vs Deep RL TD3
run_project('full');     % Model C: Complete 8-Phase Locomotion
```

---

## 2. System Architecture & Directory Organization

```
Quadrupled_Project/
│
├── MATLAB/
│   └── Core/
│       ├── Gait/             # compute_cpg_phases.m
│       ├── Kinematics/       # generate_foot_trajectory.m, solve_inverse_kinematics.m
│       ├── Balance/          # compute_pi_balance_step.m, couple_gait_and_balance.m
│       └── Dynamics/         # get_pitch_dynamics_model.m
│
├── Shared/
│   └── Configuration/
│       ├── robot_config.m       # Central hardware, link lengths, inertia, joint limits
│       ├── gait_config.m        # Stride frequency, step length, clearance, CPG offsets
│       ├── controller_config.m  # PI gains, saturation limits, neural network paths
│       └── simulation_config.m  # Step times (dt=0.005s), horizons, disturbance settings
│
├── Simulink/
│   ├── Quadruped_PID.slx              # Model A: Baseline Discrete PI/PID
│   ├── Quadruped_LSTM_TD3.slx         # Model B: Supervised LSTM & Deep RL TD3
│   ├── Quadruped_Full_Movement.slx    # Model C: Complete 8-Phase Movement Integration
│   ├── build_simulink_models.m       # Programmatic model generator
│   ├── validate_simulink_models.m    # Structural and diagram validator
│   ├── run_pid_case.m                # Model A benchmark runner
│   ├── run_lstm_td3_case.m           # Model B benchmark runner
│   └── run_full_movement_case.m      # Model C benchmark runner
│
├── Simulation_3D/
│   ├── create_3d_simulation_app.m    # Real-Time 3D GUI Dashboard
│   ├── controllers/                  # init_3d_simulation.m, step_3d_simulation.m
│   ├── robot_description/            # quadruped_robot.urdf
│   ├── world/                        # conveyor_obstacle_world.sdf
│   └── scenarios/                    # run_all_3d_scenarios.m
│
├── Documentation/
│   ├── ARCHITECTURE.md          # System architecture & tier separation
│   ├── PHASE_MAPPING.md         # Original phase to modular function & subsystem mapping
│   ├── SIGNAL_INTERFACE.md      # Signal names, dimensions, units, and data types
│   ├── CONFIGURATION_GUIDE.md   # Parameter catalog & override policies
│   ├── SIMULINK_GUIDE.md        # How to open, inspect, and simulate .slx models
│   ├── TEST_REPORT.md           # Automated test harness outcomes
│   └── PROJECT_STATUS.md        # Component completion registry
│
├── LiveScripts/
│   └── Project_Demonstration.m  # Interactive demonstration script
│
├── build_project.m              # Project builder & dependency checker
├── run_project.m                # Master interactive & programmatic launcher
├── run_phase.m                  # Dedicated isolated phase runner (Phases 1–8)
└── run_tests.m                  # Automated regression & unit test suite
```

---

## 3. Key Design Principles

1. **Strict Logic Separation**: Pure mathematical calculations in `MATLAB/Core/` do not clear the workspace, close figures, or create unexpected global state.
2. **Zero Hardcoded Constants**: Reusable parameters are centralized in `Shared/Configuration/` with strict precedence rules and input validation.
3. **Synchronous Simulink Models**: Saved `.slx` models use discrete sample times ($200\text{ Hz}$) matching physical hardware loop rates.
4. **Preserved Working Code**: All original frozen Phase 1–8 scripts remain intact and untouched, guaranteed by automated regression testing.
