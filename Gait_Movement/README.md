# Gait Movement Subsystem Architecture

This directory houses the progressive phases of gait trajectory synthesis and leg kinematics:

- `Phase_0_Interface/`: Standardized `gaitState` definition (`init_gait_state.m`).
- `Phase_1_Gait_Model/`: Analytical duty cycle and leg timing models.
- `Phase_2_Foot_Trajectory/`: Bezier and cycloidal foot swing curves.
- `Phase_3_Inverse_Kinematics/`: Analytical 3-DOF leg inverse kinematics.
- `Phase_4_Single_Leg/`: Single-leg dynamics and force mapping.
- `Phase_5_Gait_Generator/`: Central Pattern Generator (CPG) & finite state machine.
- `Phase_6_Quadruped_Walking/`: Full 4-leg walking locomotion engine.
- `Phase_7_Velocity_Control/`: Body linear/angular velocity tracking.
- `Phase_8_Turning/`: Yaw steering & differential gait adaptation.
- `Results/`: Exported trajectory metrics & validation plots.
