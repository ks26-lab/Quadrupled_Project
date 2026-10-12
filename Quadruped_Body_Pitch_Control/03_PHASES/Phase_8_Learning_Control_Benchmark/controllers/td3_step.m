function [u, ctrl] = td3_step(ctrl, theta_d, theta_m)
% TD3_STEP Single-step deterministic continuous actor evaluation.
%
% Actor network maps state [e; e_dot; theta_m; integral_e] to continuous torque u.
%
% Network layers:
%   z1 = ReLU(W1 * s + b1)
%   z2 = ReLU(W2 * z1 + b2)
%   a  = tanh(W3 * z2 + b3)
%   u  = a * u_sat

e = theta_d - theta_m;

if ~ctrl.has_prev
    e_dot = 0.0;
    ctrl.has_prev = true;
else
    e_dot = (e - ctrl.prev_e) / ctrl.dt;
end
ctrl.prev_e = e;

% Bounded integrator state
ctrl.integral_e = max(-2.0, min(2.0, ctrl.integral_e + e * ctrl.dt));

s_raw = [e; e_dot; theta_m; ctrl.integral_e];

actor = ctrl.actor;
if isfield(actor, 'mu_s') && isfield(actor, 'sigma_s')
    s = (s_raw - actor.mu_s) ./ (actor.sigma_s + 1e-8);
else
    s = s_raw;
end

% Forward pass through MLP actor
z1 = max(0, actor.W1 * s + actor.b1);
z2 = max(0, actor.W2 * z1 + actor.b2);
a  = tanh(actor.W3 * z2 + actor.b3);

u_unsat = a * ctrl.u_sat;

% Saturation safeguard
u = max(-ctrl.u_sat, min(ctrl.u_sat, u_unsat));
ctrl.last_u = u;
end
