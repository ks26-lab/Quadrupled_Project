function params = get_dynamics_and_actuator_limits()
% GET_DYNAMICS_AND_ACTUATOR_LIMITS Central authoritative parameter database
% for physical dynamics, servo limits, and gait speed optimization.
%
% Explicit Classification:
%   [M] = Manufacturer Specification / Datasheet
%   [E] = Experimentally Measured / Frozen Code
%   [S] = Simulation Assumption (Requires Hardware Verification)
%   [U] = Unknown (Must be measured on physical testbench)

root_dir = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(root_dir, 'Robot_Movement', 'Robot_Parameters'));
geom = get_quadruped_geometry();

%% 1. Physical Robot Model
params.mass_total = 5.0;            % [S] Total robot mass [kg] (from Master_Robot.m L43)
params.g          = 9.81;           % [M] Gravity acceleration [m/s^2]
params.W_total    = params.mass_total * params.g; % Total weight = 49.05 N

% Geometric Dimensions
params.body_length = geom.body.length; % [E] 0.40 m
params.body_width  = geom.body.width;  % [E] 0.20 m
params.body_height = geom.body.height; % [E] 0.12 m
params.L1          = geom.leg.hipToKnee;  % [E] 0.12 m (Thigh)
params.L2          = geom.leg.kneeToFoot; % [E] 0.15 m (Shank)
params.standing_h  = geom.standingHeight; % [E] 0.22 m

% Pitch Inertia and Damping (Frozen Baseline)
params.J_pitch     = geom.J;        % [E] 0.05 kg.m^2
params.B_pitch     = geom.B;        % [E] 0.10 N.m.s/rad
params.K_pitch     = geom.K;        % [E] 0.00 N.m/rad

%% 2. Servo & Actuator Limits (Typical 12-DOF Robot Servos, e.g., RDS3225 / STS3215 class)
% From Master_Robot.m L310: actuator.maxJointVelocity = 5.0 rad/s
params.actuator.omega_max      = 5.0;   % [S] Max no-load angular velocity [rad/s] (~47.7 rpm, 0.12 s/60deg)
params.actuator.tau_stall      = 2.5;   % [S] Stall torque [N.m] (approx 25 kg.cm at 7.4V)
params.actuator.tau_rated      = 0.80;  % [S] Continuous thermal rated torque [N.m]
params.actuator.voltage_nom    = 7.4;   % [M] Nominal supply voltage [V] (2S LiPo)
params.actuator.current_stall  = 3.0;   % [M] Stall current per servo [A]
params.actuator.current_idle   = 0.10;  % [M] Idle current per servo [A]
params.actuator.k_t            = params.tau_stall / params.actuator.current_stall; % Torque constant [N.m/A] ~ 0.833
params.actuator.max_angle      = pi;    % [S] Position limit [rad] (from Master_Robot.m L311)
params.actuator.min_angle      = -pi;   % [S] Position limit [rad]

%% 3. Power Supply and Battery Constraints
params.power.battery_voltage   = 7.4;   % [M] Nominal 2S LiPo voltage [V]
params.power.battery_capacity  = 5.0;   % [S] 5000 mAh (5.0 Ah)
params.power.max_continuous_current = 30.0; % [M] Pack discharge limit [A] (e.g. 25C 5000mAh)

%% 4. Contact & Terrain Assumptions
params.contact.mu_friction     = 0.60;  % [S] Rubber foot on indoor tile/wood (friction coefficient)
params.contact.restitution     = 0.00;  % [S] Inelastic contact assumption

%% 5. Baseline Gait Parameters
params.baseline.gait_type      = "TROT";% [E] Trot gait
params.baseline.frequency      = 1.0;   % [E] 1.0 Hz (Period T = 1.0 s)
params.baseline.step_length    = 0.10;  % [E] 0.10 m (Stride length)
params.baseline.step_height    = 0.04;  % [E] 0.04 m (Foot clearance)
params.baseline.duty_factor    = 0.50;  % [E] 50% stance, 50% swing
params.baseline.speed          = params.baseline.step_length * params.baseline.frequency; % 0.10 m/s

%% 6. Identification of Unknown Physical Parameters (Must be measured on hardware)
params.unknowns = { ...
    'Individual leg link masses and center of mass locations (m_thigh, m_shank)', ...
    'Motor armature resistance R_a and back-EMF constant k_e', ...
    'Gearbox friction, gear backlash, and transmission efficiency eta_g', ...
    'Actual foot-ground contact compliance and coefficient of friction mu', ...
    'Dynamic battery voltage sag under peak 12-servo transient current demand', ...
    'Servo thermal dissipation time constant tau_th and maximum casing temperature' ...
};

end
