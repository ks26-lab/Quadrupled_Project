function cfg = gait_config(overrides)
% GAIT_CONFIG Central authoritative locomotion & gait parameters.
%
% Configuration fields:
%   - frequency: Stride frequency [Hz] (nominal 1.0)
%   - stepLength: Forward translation per stride [m] (nominal 0.10)
%   - stepHeight: Vertical foot clearance during swing [m] (nominal 0.04)
%   - dutyFactor: Proportion of cycle in stance phase [0..1] (nominal 0.50 for trot)
%   - phaseOffsets: CPG phase offsets [4 x 1] for [LF; RF; LH; RH]
%   - velocity: Commanded velocities [vx; vy; wz]
%   - mode: Locomotion mode ('STAND', 'WALK', 'TURN')
%
% Usage:
%   cfg = gait_config()
%   cfg = gait_config(overrides)

if nargin < 1 || isempty(overrides)
    overrides = struct();
end

cfg.frequency   = 1.0;   % Stride rate in Hz
cfg.period      = 1.0 / cfg.frequency;
cfg.stepLength  = 0.10;  % Forward stride distance (m)
cfg.stepHeight  = 0.04;  % Swing ground clearance (m)
cfg.dutyFactor  = 0.50;  % Trot: 50% swing, 50% stance

% Trot CPG phase offsets for [LF; RF; LH; RH]
cfg.phaseOffsets = [
    0.0; % LF
    0.5; % RF
    0.5; % LH
    0.0  % RH
];

% Motion commands
cfg.cmd.vx = 0.10; % Forward velocity (m/s)
cfg.cmd.vy = 0.00; % Lateral velocity (m/s)
cfg.cmd.wz = 0.00; % Yaw turning rate (rad/s)

% Determine mode
if abs(cfg.cmd.wz) > 1e-4
    cfg.mode = "TURN";
elseif abs(cfg.cmd.vx) > 1e-4 || abs(cfg.cmd.vy) > 1e-4
    cfg.mode = "WALK";
else
    cfg.mode = "STAND";
end

% Balance Adaptation thresholds (rad)
cfg.thresholds.E_MILD   = 0.05;  % ~2.8 deg
cfg.thresholds.E_SEVERE = 0.15;  % ~8.6 deg

% Override support
fields = fieldnames(overrides);
for i = 1:length(fields)
    f = fields{i};
    if isfield(cfg, f)
        cfg.(f) = overrides.(f);
    end
end

assert(cfg.frequency > 0, 'Gait frequency must be strictly positive.');
assert(cfg.dutyFactor > 0 && cfg.dutyFactor < 1, 'Duty factor must be in (0, 1).');
end
