# Unified Quadruped Gait Movement Simulink Architecture

## 1. Overview
The `Simulink_Gait/` module establishes the unified quadruped gait-movement architecture for the robot project. It integrates the gait cycle, foot trajectory generation, analytical inverse kinematics, body movement, disturbance generation, and coupling to the existing SISO balance controller into a cohesive system.

All existing baseline code in `Balance_Control/`, `Gait_Movement/`, `Robot_Movement/`, and `Integration/` remains **completely frozen and unmodified**.

---

## 2. Directory Layout
```
Simulink_Gait/
├── build_gait_model.m            # Programmatic Simulink builder (.slx generator)
├── initialize_gait_model.m       # Central parameter loader
├── run_gait_simulation.m         # Discrete simulation execution engine
├── validate_gait_model.m         # 11-test verification harness
├── README.md                     # Architecture specification & usage manual
└── adapters/
    ├── cpg_gait_generator.m      # 4-leg Trot phase oscillator
    ├── four_leg_foot_trajectory.m# Instantaneous 3D foot coordinate generator
    ├── four_leg_ik_solver.m      # 12-DOF analytical leg inverse kinematics
    ├── gait_balance_coupler.m    # q_command = q_gait + delta_q_balance coupler
    └── balance_adaptation_feedback.m # Pitch-error-based gait adaptation
```

---

## 3. Subsystem Architecture

```
                       [ Simulation Time t ]
                                 │
                                 ▼
                 ┌───────────────────────────────┐
                 │   CPG Gait Phase Generator    │
                 │   - Trot offsets: [0, 0.5,...]│
                 │   - Frequency: 1.0 Hz         │
                 └───────────────┬───────────────┘
                                 │ leg_phases [4x1]
                                 ▼
                 ┌───────────────────────────────┐
                 │ Foot Trajectory Generator     │
                 │   - Sinusoidal lift (4 cm)    │
                 │   - Stride progression (10 cm)│
                 └───────────────┬───────────────┘
                                 │ foot_positions [4x3]
                                 ▼
                 ┌───────────────────────────────┐
                 │ 12-DOF Inverse Kinematics     │
                 │   - L1 = 0.12m, L2 = 0.15m    │
                 │   - Analytical reachability   │
                 └───────────────┬───────────────┘
                                 │ q_gait [12x1]
                                 ▼
[ SISO PI Balance ] ──► ┌───────────────────────────────┐
  u_pitch [N.m]         │    Gait-Balance Coupler       │ ──► [ Final 12-DOF Joint Commands ]
                        │ q_cmd = q_gait + delta_q_bal  │
                        └───────────────────────────────┘
```

---

## 4. Leg Naming and Joint Ordering
- **Leg Identifiers:**
  - `LF`: Left Front (Index 1) — Offset 0.0
  - `RF`: Right Front (Index 2) — Offset 0.5
  - `LH`: Left Hind (Index 3) — Offset 0.5
  - `RH`: Right Hind (Index 4) — Offset 0.0
- **12-DOF Joint Command Mapping:**
  - Indices 1–3: $LF$ $[q_1\text{ (roll)}, q_2\text{ (pitch)}, q_3\text{ (knee)}]$
  - Indices 4–6: $RF$ $[q_1\text{ (roll)}, q_2\text{ (pitch)}, q_3\text{ (knee)}]$
  - Indices 7–9: $LH$ $[q_1\text{ (roll)}, q_2\text{ (pitch)}, q_3\text{ (knee)}]$
  - Indices 10–12: $RH$ $[q_1\text{ (roll)}, q_2\text{ (pitch)}, q_3\text{ (knee)}]$

---

## 5. Execution and Validation

### Step 1: Initialize Parameters
```matlab
addpath('Simulink_Gait');
gait_cfg = initialize_gait_model();
```

### Step 2: Build the Simulink Model (.slx)
In a MATLAB environment with Simulink:
```matlab
build_gait_model();
```
This generates and saves `Quadruped_Gait_Unified.slx` programmatically.

### Step 3: Run the 11-Test Verification Suite
```matlab
test_report = validate_gait_model();
```

### Verified Test Suite:
1. **TEST 1: Initialization** — Verifies geometry and link dimensions match authoritative sources ($L_1=0.12, L_2=0.15$).
2. **TEST 2: Gait Phase Progression** — Verifies linear phase advance and boundary modulo wrapping.
3. **TEST 3: Four-Leg Coordination** — Verifies diagonal trot pairing ($\{LF, RH\}$ vs $\{RF, LH\}$).
4. **TEST 4: Foot Trajectory** — Confirms exact clearance peak ($4.0\,\text{cm}$) and stance adherence.
5. **TEST 5: Inverse Kinematics** — Cross-validates against baseline `solve_leg_ik.m` ($<10^{-10}\,\text{rad}$ difference).
6. **TEST 6: Joint Constraints** — Tests reachability flags under out-of-workspace targets.
7. **TEST 7: Robot Movement Integration** — Confirms 12-DOF telemetry stream.
8. **TEST 8: Balance Integration** — Confirms SISO pitch balance updates pitch joints only, preserving roll/knee angles.
9. **TEST 9: Disturbance Interface** — Injects $0.10\,\text{N}\cdot\text{m}$ transient pulse.
10. **TEST 10: Repeatability** — Confirms double-run numerical determinism ($0.00\,\text{rad}$ difference).
11. **TEST 11: Regression** — Verifies all frozen baseline scripts exist and remain unmodified.
