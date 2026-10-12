function ctrl = pi_baseline_controller(cfg)
% PI_BASELINE_CONTROLLER Discrete-time PI controller with anti-windup & saturation.
%
% Baseline conventional controller matching the frozen parameters:
%   Kp = 0.50, Ki = 0.20 (or Ki = 0.17 if specified by cfg)
%
% State struct ctrl holds:
%   - Kp, Ki, dt, u_sat, integrator, last_u, anti_windup_enabled

if nargin < 1, cfg = struct(); end

if isfield(cfg, 'Kp'), ctrl.Kp = cfg.Kp; else, ctrl.Kp = 0.50; end
if isfield(cfg, 'Ki'), ctrl.Ki = cfg.Ki; else, ctrl.Ki = 0.20; end
if isfield(cfg, 'dt'), ctrl.dt = cfg.dt; else, ctrl.dt = 0.005; end
if isfield(cfg, 'u_sat'), ctrl.u_sat = cfg.u_sat; else, ctrl.u_sat = 0.40; end
if isfield(cfg, 'anti_windup'), ctrl.anti_windup = cfg.anti_windup; else, ctrl.anti_windup = true; end

ctrl.integrator = 0.0;
ctrl.last_u = 0.0;
ctrl.type = 'PI';
end
