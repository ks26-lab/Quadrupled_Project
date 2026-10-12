# Movement Phase Mapping & Integration Guide

## Overview
This document maps each of the original frozen Movement Phases 1 through 8 to its corresponding modular function in `MATLAB/Core/` and its subsystem block in Simulink.

| Phase | Phase Title | Original Frozen Location | New Modular MATLAB Function | Simulink Subsystem Block |
| :---: | :--- | :--- | :--- | :--- |
| **Phase 1** | Gait Interface & Initialization | `Gait_Movement/Phase_0_Interface/phase_1_interface.m` | `Shared/Configuration/gait_config.m` | `Phase1_8_Motion_Command` |
| **Phase 2** | Robot Geometry & Frame Origins | `Robot_Movement/Robot_Parameters/phase_2_robot_geometry.m` | `Shared/Configuration/robot_config.m` | `Phase2_Robot_Geometry` |
| **Phase 3** | Foot Trajectory Generation | `Gait_Movement/Phase_2_Foot_Trajectory/phase_3_foot_trajectory.m` | `MATLAB/Core/Kinematics/generate_foot_trajectory.m` | `Phase3_7_Foot_Trajectory` |
| **Phase 4** | 12-DOF Inverse Kinematics | `Gait_Movement/Phase_3_Inverse_Kinematics/phase_4_inverse_kinematics.m` | `MATLAB/Core/Kinematics/solve_inverse_kinematics.m` | `Phase4_5_Inverse_Kinematics` |
| **Phase 5** | Single-Leg Motion & Validation | `Gait_Movement/Phase_4_Single_Leg/run_phase_5_single_leg.m` | `run_phase(5)` | `Phase4_5_Inverse_Kinematics/q_lead_leg` |
| **Phase 6** | Coordinated 4-Leg Trot Gait | `Gait_Movement/Master_Gait.m` (Lines 108–158) | `MATLAB/Core/Gait/compute_cpg_phases.m` | `Phase6_CPG_Coordinator` |
| **Phase 7** | Forward Walking Trajectory | `Gait_Movement/Master_Gait.m` (Lines 160–220) | `MATLAB/Core/Kinematics/generate_foot_trajectory.m` | `Phase3_7_Foot_Trajectory` |
| **Phase 8** | Velocity Control & Turning | `Gait_Movement/Master_Gait.m` (Lines 222–270) | `Shared/Configuration/gait_config.m` | `Phase1_8_Motion_Command` |
| **Balance** | Frozen PI Body Pitch Control | `Quadruped_Body_Pitch_Control/03_PHASES/Phase_4_PI_Controller/` | `MATLAB/Core/Balance/compute_pi_balance_step.m` | `Balance_Pitch_Control_Subsystem` |
| **Coupling**| Gait-Balance Joint Combination | `Integration/Gait_Balance_Interface.m` | `MATLAB/Core/Balance/couple_gait_and_balance.m` | `Gait_Balance_Interface` |
| **Adaptation**| Terrain Pitch Error Feedback | `Integration/Balance_Gait_Interface.m` | `Simulation_3D/controllers/step_3d_simulation.m` | `Balance_Gait_Adaptation` |

---

## Independent Phase Execution
Each phase can be tested in isolation using the dedicated top-level runner:
```matlab
run_phase(1); % Test state configuration
run_phase(2); % Verify robot geometry
run_phase(3); % Generate foot trajectory
run_phase(4); % Solve 12-DOF IK
run_phase(5); % Verify single-leg limits
run_phase(6); % Verify 4-leg CPG coordination
run_phase(7); % Generate forward walking sequence
run_phase(8); % Arbitrate motion commands
```
