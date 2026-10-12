function gait_cfg = initialize_gait_model()
% INITIALIZE_GAIT_MODEL Central parameter loader for Quadruped_Gait_Unified.
%
% Reuses authoritative geometry, gait configuration, and baseline balance parameters
% without duplicating or hard-coding them.

root_dir = fileparts(fileparts(mfilename('fullpath')));

% Add dependencies dynamically
addpath(fullfile(root_dir, 'Gait_Movement'));
addpath(fullfile(root_dir, 'Gait_Movement', 'Phase_0_Interface'));
addpath(fullfile(root_dir, 'Gait_Movement', 'Phase_3_Inverse_Kinematics'));
addpath(fullfile(root_dir, 'Robot_Movement', 'Robot_Parameters'));
addpath(fullfile(root_dir, 'Integration'));

% 1. Authoritative Physical & Geometric Parameters
geom = get_quadruped_geometry();
gait_cfg.geom = geom;

gait_cfg.L1 = geom.leg.hipToKnee;    % 0.12 m
gait_cfg.L2 = geom.leg.kneeToFoot;   % 0.15 m
gait_cfg.hipPosition = geom.hipPosition; % [4 x 3]
gait_cfg.nominalFootPosition = geom.nominalFootPosition; % [4 x 3]

% 2. Authoritative Gait Timing and Motion Parameters
gait_cfg.frequency   = 1.0;   % 1.0 Hz (Period T = 1.0 s)
gait_cfg.T           = 1.0 / gait_cfg.frequency;
gait_cfg.stepLength  = 0.10;  % 0.10 m
gait_cfg.stepHeight  = 0.04;  % 0.04 m
gait_cfg.dutyFactor  = 0.50;  % Trot duty cycle (50% swing, 50% stance)

% Trot phase offsets for [LF, RF, LH, RH]
gait_cfg.phaseOffsets = [0.0; 0.5; 0.5; 0.0];

% 3. Motion Command Default Settings
gait_cfg.vx_cmd = 0.10; % m/s
gait_cfg.vy_cmd = 0.00; % m/s
gait_cfg.wz_cmd = 0.00; % rad/s

% 4. Hardware and Simulation Sampling
gait_cfg.dt = 0.005; % 200 Hz loop rate (0.005 s)
gait_cfg.t_final = 5.0;

% 5. Frozen SISO Balance Controller Parameters (Preserved)
gait_cfg.balance.Kp = 0.50;
gait_cfg.balance.Ki = 0.20;
gait_cfg.balance.u_sat = 0.40; % N.m
gait_cfg.balance.J = geom.J;   % 0.05
gait_cfg.balance.B = geom.B;   % 0.10
gait_cfg.balance.K = geom.K;   % 0.00
gait_cfg.balance.k_torque_to_q = 0.05;

% 6. Initial States
gait_cfg.init.theta = 0.0;     % pitch [rad]
gait_cfg.init.theta_dot = 0.0; % pitch rate [rad/s]
gait_cfg.init.integrator = 0.0;
gait_cfg.init.body_pos = [0; 0; geom.standingHeight];

% Assign into base workspace for Simulink block access
assignin('base', 'gait_cfg', gait_cfg);
fprintf('[Simulink Gait] Loaded authoritative configuration into base workspace.\n');
end
