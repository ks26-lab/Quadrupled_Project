# Quadruped Robot System Architecture & Modular Design

## 1. Architectural Philosophy
The codebase is structured around a strict separation of concerns across five primary tiers:
1. **Computational Tier (`MATLAB/Core/`)**: Pure, side-effect-free mathematical algorithms (Kinematics, CPG, Dynamics, Control equations).
2. **Control & Model Tier (`Simulink/`)**: Native block-diagram architectures containing executable discrete controllers, plants, and buses.
3. **Configuration & Data Contract Tier (`Shared/Configuration/`)**: Authoritative, single-source-of-truth hardware dimensions, inertia, sample times, and gains.
4. **Integration & Simulation Tier (`Simulation_3D/`)**: Real-time 3D conveyor physics, obstacle collision geometry, and interactive visual dashboards.
5. **Testing & Validation Tier (`run_tests.m`)**: Automated regression and unit test harnesses.

```
       +---------------------------------------------+
       |   Shared Configuration (Single Source)      |
       |  (robot_config, gait_config, controller_cfg)|
       +---------------------------------------------+
                              |
              +---------------+---------------+
              |                               |
              v                               v
    +-------------------+           +-------------------+
    |    MATLAB Core    |           |     Simulink      |
    | Pure Mathematical |           | Executable Models |
    |   Algorithms      |           |     (.slx)        |
    +-------------------+           +-------------------+
              |                               |
              +---------------+---------------+
                              |
                              v
       +---------------------------------------------+
       |   Simulation_3D Real-Time Physics & GUI     |
       |  (Conveyor belt, Obstacles, Live Telemetry) |
       +---------------------------------------------+
```

---

## 2. Directory Layout & Module Responsibilities

### `Shared/Configuration/`
- **`robot_config.m`**: Link lengths ($L_1=0.12\text{ m}, L_2=0.15\text{ m}$), body mass ($5.0\text{ kg}$), inertia ($J=0.05\text{ kg}\cdot\text{m}^2$), joint limits.
- **`gait_config.m`**: Stride frequency ($1.0\text{ Hz}$), step length ($0.10\text{ m}$), clearance height ($0.04\text{ m}$), trot offsets.
- **`controller_config.m`**: Gains ($K_p=0.50, K_i=0.20$), anti-windup, saturation ($u_{\text{sat}}=0.40\text{ N}\cdot\text{m}$), neural network paths.
- **`simulation_config.m`**: Time step ($\Delta t=0.005\text{ s}$ / $200\text{ Hz}$), horizon ($5.0\text{ s}$), disturbance profiles.

### `MATLAB/Core/`
- **`Gait/compute_cpg_phases.m`**: Evaluates continuous normalized phases $p_i(t) \in [0, 1)$ and contact booleans.
- **`Kinematics/generate_foot_trajectory.m`**: Synthesizes 3D cycloidal swing curves and linear stance retraction.
- **`Kinematics/solve_inverse_kinematics.m`**: Closed-form analytical 12-DOF IK with reachability verification.
- **`Balance/compute_pi_balance_step.m`**: Discrete-time pitch error correction with conditional clamping.
- **`Balance/couple_gait_and_balance.m`**: Enforces $q_{\text{command}} = q_{\text{gait}} + \Delta q_{\text{balance}}$.
- **`Dynamics/get_pitch_dynamics_model.m`**: Analytical plant $G(s) = \frac{1}{J s^2 + B s + K}$.

### `Simulink/`
- **`Quadruped_PID.slx`**: Model A baseline discrete pitch stabilization model.
- **`Quadruped_LSTM_TD3.slx`**: Model B dual learning-based controller model (TD3 Continuous Actor & Supervised LSTM).
- **`Quadruped_Full_Movement.slx`**: Model C full 8-phase movement and balance integration.
- **`build_simulink_models.m`**: Programmatic model generator.
- **`validate_simulink_models.m`**: Automated structural and compilation validator.

### `Simulation_3D/`
- **`create_3d_simulation_app.m`**: Interactive GUI dashboard with 3D viewport, conveyor belt animation, and telemetry plots.
- **`controllers/step_3d_simulation.m`**: 200 Hz physics update loop resolving conveyor friction and obstacle contact.
- **`robot_description/quadruped_robot.urdf`**: Authoritative URDF description for ROS 2 / Gazebo Sim.
- **`world/conveyor_obstacle_world.sdf`**: Physical world SDF definition.
