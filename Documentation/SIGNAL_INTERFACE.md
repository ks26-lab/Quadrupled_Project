# MATLAB–Simulink Signal Interface Specification

## 1. Top-Level Data Contract
Simulink models exchange signals with the MATLAB layer using standardized structures and logged arrays.

| Signal Name | Description | Dimensions | Units | Data Type | Sample Time |
| :--- | :--- | :---: | :---: | :---: | :---: |
| `theta_d` | Commanded body pitch angle | Scalar | $\text{rad}$ | `double` | Continuous / $0.005\text{ s}$ |
| `theta` | Actual body pitch angle | Scalar | $\text{rad}$ | `double` | Continuous / $0.005\text{ s}$ |
| `theta_dot` | Body pitch angular velocity | Scalar | $\text{rad/s}$ | `double` | Continuous / $0.005\text{ s}$ |
| `pitch_error` | Pitch tracking error ($e = \theta_d - \theta$) | Scalar | $\text{rad}$ | `double` | $0.005\text{ s}$ ($200\text{ Hz}$) |
| `u_pitch` | Pitch control torque | Scalar | $\text{N}\cdot\text{m}$ | `double` | $0.005\text{ s}$ ($200\text{ Hz}$) |
| `tau_dist` | External disturbance torque | Scalar | $\text{N}\cdot\text{m}$ | `double` | $0.005\text{ s}$ ($200\text{ Hz}$) |
| `leg_phases` | CPG phase offsets for 4 legs | $[4 \times 1]$ | $[0, 1)$ | `double` | $0.005\text{ s}$ ($200\text{ Hz}$) |
| `stance_states` | Ground contact support flags | $[4 \times 1]$ | Boolean | `boolean` | $0.005\text{ s}$ ($200\text{ Hz}$) |
| `foot_positions` | 3D Foot coordinates in hip frames | $[4 \times 3]$ | $\text{m}$ | `double` | $0.005\text{ s}$ ($200\text{ Hz}$) |
| `q_gait` | Commanded joint angles from IK | $[12 \times 1]$ | $\text{rad}$ | `double` | $0.005\text{ s}$ ($200\text{ Hz}$) |
| `delta_q_balance`| Hip pitch balance corrections | $[12 \times 1]$ | $\text{rad}$ | `double` | $0.005\text{ s}$ ($200\text{ Hz}$) |
| `q_command` | Net commanded joint angles | $[12 \times 1]$ | $\text{rad}$ | `double` | $0.005\text{ s}$ ($200\text{ Hz}$) |

---

## 2. Joint Index Ordering (12-DOF Vector)
The 12-DOF joint vector `q_command` follows the authoritative leg ordering:
- `LF (Left Front)`:
  - `q(1)`: Hip roll (abduction/adduction)
  - `q(2)`: Hip pitch (thigh)
  - `q(3)`: Knee pitch (shank)
- `RF (Right Front)`:
  - `q(4)`: Hip roll
  - `q(5)`: Hip pitch
  - `q(6)`: Knee pitch
- `LH (Left Hind)`:
  - `q(7)`: Hip roll
  - `q(8)`: Hip pitch
  - `q(9)`: Knee pitch
- `RH (Right Hind)`:
  - `q(10)`: Hip roll
  - `q(11)`: Hip pitch
  - `q(12)`: Knee pitch
