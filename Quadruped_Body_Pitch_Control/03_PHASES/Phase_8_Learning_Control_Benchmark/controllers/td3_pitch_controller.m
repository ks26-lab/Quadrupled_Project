function ctrl = td3_pitch_controller(policy_or_mat, cfg)
% TD3_PITCH_CONTROLLER Continuous Actor Policy Controller Constructor.
%
% Wraps a trained Continuous Actor policy (from TD3 or Reinforcement Learning).
% Input state representation to Actor: s = [e; e_dot; theta_m; integral_e]
%
% Architecture:
%   Linear(in_dim -> h1) -> ReLU -> Linear(h1 -> h2) -> ReLU -> Linear(h2 -> 1) -> tanh -> * u_sat

if nargin < 2, cfg = struct(); end

if isfield(cfg, 'dt'), ctrl.dt = cfg.dt; else, ctrl.dt = 0.005; end
if isfield(cfg, 'u_sat'), ctrl.u_sat = cfg.u_sat; else, ctrl.u_sat = 0.40; end

if ischar(policy_or_mat) || isstring(policy_or_mat)
    if exist(policy_or_mat, 'file')
        loaded = load(policy_or_mat);
        if isfield(loaded, 'actor'), ctrl.actor = loaded.actor;
        elseif isfield(loaded, 'policy'), ctrl.actor = loaded.policy;
        else, ctrl.actor = loaded;
        end
    else
        error('TD3 policy file not found: %s', policy_or_mat);
    end
elseif isstruct(policy_or_mat)
    ctrl.actor = policy_or_mat;
else
    error('policy_or_mat must be a filepath or struct.');
end

ctrl.prev_e = 0.0;
ctrl.integral_e = 0.0;
ctrl.has_prev = false;
ctrl.last_u = 0.0;
ctrl.type = 'TD3';
end
