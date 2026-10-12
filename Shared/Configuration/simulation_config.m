function cfg = simulation_config(overrides)
% SIMULATION_CONFIG Central simulation solver, horizon, and disturbance settings.
%
% Holds:
%   - dt: Fundamental step time [0.005 s]
%   - t_end: Duration [5.0 s]
%   - solver: 'FixedStepDiscrete' or ODE solver
%   - disturbance: Default pulse (0.10 N.m from 2.0s to 2.5s)
%
% Usage:
%   cfg = simulation_config()
%   cfg = simulation_config(overrides)

if nargin < 1 || isempty(overrides)
    overrides = struct();
end

cfg.dt = 0.005; % 200 Hz loop rate
cfg.t_end = 5.0; % 5.0 s nominal horizon
cfg.time_vec = 0:cfg.dt:cfg.t_end;
cfg.num_steps = length(cfg.time_vec);

cfg.solver = 'FixedStepDiscrete';
cfg.rel_tol = 1e-4;
cfg.abs_tol = 1e-6;

% Nominal Disturbance Settings
cfg.disturbance.type = 'transient_pulse';
cfg.disturbance.start_time = 2.0; % s
cfg.disturbance.end_time   = 2.5; % s
cfg.disturbance.magnitude  = 0.10; % N.m

% Overrides
fields = fieldnames(overrides);
for i = 1:length(fields)
    f = fields{i};
    if isfield(cfg, f)
        cfg.(f) = overrides.(f);
    end
end

assert(cfg.dt > 0 && cfg.t_end > cfg.dt, 'Simulation timing parameters invalid.');
end
