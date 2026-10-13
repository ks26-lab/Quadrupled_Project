# Dynamic Simulation Audit and Correction — supplied MATLAB module

## Scope and limitations

Inspected the user-supplied `create_3d_simulation_app.m`, `init_3d_simulation.m`, and `step_3d_simulation.m` source. The complete local repository and `get_quadruped_geometry.m` were not supplied in this turn, and MATLAB/Simulink is unavailable in this environment. These corrections are **source-reviewed, not MATLAB-executed**. This is a **reduced-order demonstration**, not a physically validated multibody or contact dynamics model.

## Trace of runtime data

`init_3d_simulation(cfg)` builds configuration, obstacles, state, and empty history. The app's timer calls `step_3d_simulation` five times per nominal 25 ms frame (subject to configured timing), then reads `engine.state` to position graphics and `engine.history` to plot pitch/torque. Callbacks modify the same `engine` state. No fabricated history samples are seeded.

## Findings and changes

| Source and location | Original behavior / defect | Correction | Verification |
|---|---|---|---|
| `init_3d_simulation`, argument `cfg` | Accepted but unused | Validated overrides for robot, conveyor, gait, time, physics, obstacles | Override test |
| `init_3d_simulation`, obstacles | Heights hardwired to belt at 0.20 m | Default obstacle centres rebased if configured belt elevation changes | Configuration/terrain test |
| `create_3d_simulation_app`, obstacle loop | Most obstacles shown as single top quad | Solid step/block six-face mesh, ramp wedge, patch thin overlay | Visual inspection required |
| `create_3d_simulation_app`, ramp | Ignores configured Y/Z and constructs malformed surface | Mesh uses obstacle centre and dimensions; slope rises +X | Ramp endpoints test |
| `step_3d_simulation`, obstacle loop | Patch erroneously treated as raised terrain | `terrain_at_xy` changes friction only for patch | Friction-patch test |
| `step_3d_simulation`, contact force | Adds 50 N without penetration and discards result | Removed fabricated force; spring-damper diagnostic logged | Nonnegative force test; deeper physical test pending |
| `step_3d_simulation`, world foot positions | Foot positions calculated before final body update | Recomputed after body pose update | State/renderer consistency review |
| `step_3d_simulation`, contact and pitch | Terrain height changes pitch via heuristic moment | Retained explicitly labeled configurable approximation | Numerical tests and model calibration pending |
| `step_3d_simulation`, conveyor slip | Binary contact gating, no friction dependence | Friction-dependent **heuristic** traction/slip ratio | Compare nominal vs patch runs in MATLAB |
| `create_3d_simulation_app`, conveyor | Geometry hardcoded to fixed endpoints | Belt and stripe positions derived from engine dimensions | Resize scenario and visually inspect |
| `create_3d_simulation_app`, reset/scenario | Engine reset did not recreate obstacle graphics | Rebuild scene and resync controls | Reset/scenario GUI checks |
| `create_3d_simulation_app`, disturbance | Button said 0.10 N m but wrote force=2 N | Explicit finite-duration torque in N m | Torque duration test |
| `create_3d_simulation_app`, telemetry | Initial status and limits were hardcoded strings | Status/limits derive from engine state/configuration | GUI status vs history comparison |
| `create_3d_simulation_app`, body/legs | Body top patch and midpoint knees approximate physical shape | Preserved but explicitly limited to **visual approximations** | Need forward kinematics / multibody model to validate |
| `step_3d_simulation`, vertical body dynamics | Height snapped to terrain average; force never integrated | Preserved as explicitly documented **kinematic terrain following** | Requires dynamics upgrade for physical claims |
| `step_3d_simulation`, inverse kinematics | Feasibility silently ignored | Feasibility exposed as state/status; no invented valid joint pose | IK test with geometry data |

## Important physical assumptions

- Obstacle dimensions are **preserved** from the original source: step 0.25×0.60×0.03 m; ramp 0.40×0.60×0.04 m; block 0.15×0.50×0.05 m; friction zone 0.35×0.70 m. Their centres remain at X=0.80, 1.40, 2.10, 2.70 m respectively.
- Ramp rises in +X from conveyor top to +0.04 m. Step and block rest on the conveyor. Patch is a surface coating, not a missing-support gap.
- Patch friction coefficient 0.12 is an **illustrative, configurable assumption**; no measured coefficient was provided.
- `nominalFootPosition` frame convention and leg kinematics require comparison with `get_quadruped_geometry.m`. The original nominal-foot convention is retained rather than silently redefining coordinates.
- Normal forces are **diagnostics**, not forces integrated into the robot's vertical momentum. Body translation, terrain-following and pitch terrain torque remain simplified. The GUI must not be presented as a validated physical robot simulator.
- GUI timer provides nominal simulated step frequency, not guaranteed wall-clock real-time performance.

## How to run

Place the five `.m` files together in the project's simulation directory, backing up originals first. In MATLAB:

```matlab
addpath('/Users/umikasood/Documents/Quadrupled_Project/Simulink')
verify_dynamic_simulation
app = create_3d_simulation_app();
```

If your files live in a different simulation folder, change the `addpath` accordingly. The geometry helper must be accessible on MATLAB's path.

## Verification status

- [x] Source-level inspection and replacement files produced
- [x] Single obstacle terrain query shared by runtime collision and geometry assumptions
- [x] Deterministic MATLAB verification script written
- [ ] MATLAB unit checks executed (requires user's MATLAB)
- [ ] GUI screenshots and scenario interactions verified (requires user's MATLAB)
- [ ] Physical validation against measured contact and joint dynamics (requires robot model/data)
