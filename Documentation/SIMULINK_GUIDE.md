# Simulink Modeling & Inspection Guide

## 1. Primary Simulink Models in `Simulink/`

| Model File | Subsystem Hierarchy | Solver Configuration | Purpose |
| :--- | :--- | :--- | :--- |
| **`Quadruped_PID.slx`** | `PID_Controller`, `Robot_Pitch_Plant`, `Disturbance_Generator`, `IMU_Sensor` | Fixed-step (`FixedStepDiscrete`, $\Delta t=0.005\text{ s}$) | Baseline SISO pitch stabilization & disturbance rejection. |
| **`Quadruped_LSTM_TD3.slx`** | `Observation_Pipeline`, `TD3_Actor_Controller`, `LSTM_Recurrent_Controller`, `Policy_Selector` | Fixed-step (`FixedStepDiscrete`, $\Delta t=0.005\text{ s}$) | Machine learning benchmark (TD3 vs LSTM vs PI). |
| **`Quadruped_Full_Movement.slx`** | `Phase1_8_Motion_Command`, `Phase6_CPG_Coordinator`, `Phase3_7_Foot_Trajectory`, `Phase4_5_Inverse_Kinematics`, `Balance_Pitch_Control_Subsystem`, `Gait_Balance_Interface` | Fixed-step (`FixedStepDiscrete`, $\Delta t=0.005\text{ s}$) | Full 8-phase locomotion, kinematics, and balance coupling. |

---

## 2. Opening and Inspecting Models

From the MATLAB prompt:
```matlab
% Ensure paths are initialized
run('01_SETUP/project_setup.m');

% Open Model A
open_system('Simulink/Quadruped_PID.slx');

% Open Model B
open_system('Simulink/Quadruped_LSTM_TD3.slx');

% Open Model C
open_system('Simulink/Quadruped_Full_Movement.slx');
```

---

## 3. Programmatic Model Generation
If any `.slx` model file needs to be rebuilt or regenerated:
```matlab
cd('Simulink');
models = build_simulink_models();
```

---

## 4. Validating and Updating Diagrams
To compile and verify block connectivity without opening windows:
```matlab
cd('Simulink');
report = validate_simulink_models();
```
