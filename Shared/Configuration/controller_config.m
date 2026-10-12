function cfg = controller_config(overrides)
% CONTROLLER_CONFIG Central parameters for Baseline PI, LSTM, and TD3 Controllers.
%
% Holds:
%   - PI gains: Kp (0.50), Ki (0.20), anti-windup flag, dt (0.005s)
%   - Saturation limit: u_sat (0.40 N.m)
%   - Neural network weights paths: trained_lstm_model.mat, trained_td3_model.mat
%   - Architecture parameters: LSTM hidden_dim (16), TD3 state_dim (4)
%
% Usage:
%   cfg = controller_config()
%   cfg = controller_config(overrides)

if nargin < 1 || isempty(overrides)
    overrides = struct();
end

% 1. Baseline PI Controller Parameters
cfg.pi.Kp = 0.50;
cfg.pi.Ki = 0.20;
cfg.pi.anti_windup = true;

% Actuator saturation bound
cfg.u_sat = 0.40; % N.m

% Discrete loop sample time
cfg.dt = 0.005; % 200 Hz

% 2. Machine Learning Controller File Paths
shared_dir = fileparts(fileparts(mfilename('fullpath')));
proj_root  = fileparts(shared_dir);

cfg.lstm.weights_file = fullfile(proj_root, 'Quadruped_Body_Pitch_Control', ...
    '03_PHASES', 'Phase_8_Learning_Control_Benchmark', 'controllers', 'trained_lstm_model.mat');
cfg.lstm.hidden_dim = 16;
cfg.lstm.in_dim = 3;

cfg.td3.weights_file = fullfile(proj_root, 'Quadruped_Body_Pitch_Control', ...
    '03_PHASES', 'Phase_8_Learning_Control_Benchmark', 'controllers', 'trained_td3_model.mat');
cfg.td3.state_dim = 4;
cfg.td3.act_dim = 1;

% Apply overrides
fields = fieldnames(overrides);
for i = 1:length(fields)
    f = fields{i};
    if isfield(cfg, f)
        cfg.(f) = overrides.(f);
    end
end

assert(cfg.pi.Kp > 0, 'Proportional gain Kp must be positive.');
assert(cfg.u_sat > 0, 'Saturation bound must be strictly positive.');
end
