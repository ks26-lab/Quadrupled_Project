function cfg = robot_config(overrides)
% ROBOT_CONFIG Central authoritative hardware & geometric parameters for Quadruped.
%
% Returns a validated struct with:
%   - Link dimensions: L1 (thigh), L2 (shank), nominal standing height
%   - Body dimensions: length, width, height, total mass
%   - Hip origins in body frame: [LF; RF; LH; RH]
%   - Nominal foot coordinates in body frame: [LF; RF; LH; RH]
%   - Joint angle limits: [q1_min, q1_max; q2_min, q2_max; q3_min, q3_max]
%   - Actuator torque limit: u_sat (0.40 N.m)
%
% Usage:
%   cfg = robot_config()
%   cfg = robot_config(overrides)

if nargin < 1 || isempty(overrides)
    overrides = struct();
end

% Body dimensions (m) and mass (kg)
cfg.body.length = 0.40;
cfg.body.width  = 0.20;
cfg.body.height = 0.12;
cfg.body.mass   = 5.0;

% Pitch dynamic parameters (J_pitch in kg.m^2, B_pitch in N.m.s/rad, K_stiffness in N.m/rad)
cfg.dynamics.J = 0.05;
cfg.dynamics.B = 0.10;
cfg.dynamics.K = 0.00;

% Leg link lengths (m)
cfg.leg.L1 = 0.12; % Hip-to-knee (thigh)
cfg.leg.L2 = 0.15; % Knee-to-foot (shank)
cfg.standingHeight = 0.22; % Nominal ground clearance (m)

% Coordinate frame conventions: x = forward, y = left, z = up
cfg.frame.x = "forward";
cfg.frame.y = "left";
cfg.frame.z = "up";

% Hip joint origins in body frame [x, y, z] for [LF; RF; LH; RH]
L_half = cfg.body.length / 2;
W_half = cfg.body.width / 2;
cfg.hipPosition = [
     L_half,  W_half, 0.0; % LF
     L_half, -W_half, 0.0; % RF
    -L_half,  W_half, 0.0; % LH
    -L_half, -W_half, 0.0  % RH
];

% Nominal foot contact points relative to body center [LF; RF; LH; RH]
cfg.nominalFootPosition = [
     0.15,  0.10, -cfg.standingHeight; % LF
     0.15, -0.10, -cfg.standingHeight; % RF
    -0.15,  0.10, -cfg.standingHeight; % LH
    -0.15, -0.10, -cfg.standingHeight  % RH
];

% Joint excursion limits (rad)
% q1: hip roll [-45, +45] deg
% q2: hip pitch [-90, +90] deg
% q3: knee pitch [0, 135] deg
cfg.jointLimits = [
    -pi/4,  pi/4;
    -pi/2,  pi/2;
     0.0,   3*pi/4
];

% Actuator torque saturation limit (N.m)
cfg.actuator.u_sat = 0.40;
cfg.actuator.k_torque_to_q = 0.05; % Gain mapping pitch torque to hip pitch angle

% Apply overrides with validation
fields = fieldnames(overrides);
for i = 1:length(fields)
    f = fields{i};
    if isfield(cfg, f)
        cfg.(f) = overrides.(f);
    end
end

% Integrity assertions
assert(cfg.leg.L1 > 0 && cfg.leg.L2 > 0, 'Link lengths must be positive.');
assert(cfg.standingHeight < (cfg.leg.L1 + cfg.leg.L2), 'Standing height exceeds kinematic leg extension.');
end
