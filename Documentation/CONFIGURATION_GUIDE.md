# Configuration Guide & Parameter Override Policy

## 1. Authoritative Precedence Rules
Configuration values are loaded using a strict hierarchy:
1. **Validated Project Defaults**: Defined in `Shared/Configuration/` (`robot_config.m`, `gait_config.m`, `controller_config.m`, `simulation_config.m`).
2. **Scenario Overrides**: Passed as struct arguments to test scripts and scenario runners.
3. **User Overrides**: Explicitly provided by the user via interactive dashboard sliders or script arguments.

---

## 2. Parameter Catalog & Storage Locations

### Physical Robot Parameters (`Shared/Configuration/robot_config.m`)
| Parameter | Default Value | Units | Description |
| :--- | :---: | :---: | :--- |
| `body.length` | `0.40` | $\text{m}$ | Longitudinal chassis length |
| `body.width` | `0.20` | $\text{m}$ | Lateral chassis width |
| `body.height` | `0.12` | $\text{m}$ | Vertical chassis height |
| `body.mass` | `5.0` | $\text{kg}$ | Total robot mass |
| `dynamics.J` | `0.05` | $\text{kg}\cdot\text{m}^2$ | Body pitch moment of inertia |
| `dynamics.B` | `0.10` | $\text{N}\cdot\text{m}\cdot\text{s/rad}$ | Body pitch damping coefficient |
| `dynamics.K` | `0.00` | $\text{N}\cdot\text{m/rad}$ | Body pitch restoring stiffness |
| `leg.L1` | `0.12` | $\text{m}$ | Thigh link length (hip to knee) |
| `leg.L2` | `0.15` | $\text{m}$ | Shank link length (knee to foot) |
| `standingHeight` | `0.22` | $\text{m}$ | Nominal ground clearance |

### Gait Parameters (`Shared/Configuration/gait_config.m`)
| Parameter | Default Value | Units | Description |
| :--- | :---: | :---: | :--- |
| `frequency` | `1.0` | $\text{Hz}$ | Stride cadence rate ($T = 1.0\text{ s}$) |
| `stepLength` | `0.10` | $\text{m}$ | Forward translation per stride |
| `stepHeight` | `0.04` | $\text{m}$ | Swing vertical foot clearance |
| `dutyFactor` | `0.50` | $-$ | Trot duty cycle ($50\%$ swing, $50\%$ stance) |
| `phaseOffsets` | `[0.0; 0.5; 0.5; 0.0]` | $-$ | Phase offsets for [LF; RF; LH; RH] |

### Controller Parameters (`Shared/Configuration/controller_config.m`)
| Parameter | Default Value | Units | Description |
| :--- | :---: | :---: | :--- |
| `pi.Kp` | `0.50` | $\text{N}\cdot\text{m/rad}$ | Proportional pitch feedback gain |
| `pi.Ki` | `0.20` | $\text{N}\cdot\text{m/(rad}\cdot\text{s)}$ | Integral pitch feedback gain |
| `u_sat` | `0.40` | $\text{N}\cdot\text{m}$ | Actuator torque saturation limit |
| `dt` | `0.005` | $\text{s}$ | Discrete controller loop time ($200\text{ Hz}$) |

---

## 3. How to Override Parameters Programmatically

To run a simulation with custom parameters without modifying source files:

```matlab
% Example: Test higher cadence (1.5 Hz) and larger step clearance (6 cm)
g_overrides.frequency  = 1.5;
g_overrides.stepHeight = 0.06;
custom_gait = gait_config(g_overrides);

% Example: Test higher disturbance torque
s_overrides.disturbance.magnitude = 0.25;
custom_sim = simulation_config(s_overrides);
```
