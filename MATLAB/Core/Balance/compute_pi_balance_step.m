function [u, state] = compute_pi_balance_step(state, theta_d, theta_m, c_cfg)
% COMPUTE_PI_BALANCE_STEP Evaluates one discrete step of the frozen PI pitch controller.
%
% Implements:
%   e = theta_d - theta_m
%   u_p = Kp * e
%   u_i = Ki * integral_e
%   u   = sat(u_p + u_i, [-u_sat, +u_sat])
%   Conditional anti-windup clamping
%
% Inputs:
%   state   : Controller state struct with fields (integrator, last_u)
%   theta_d : Desired pitch reference [rad]
%   theta_m : Measured pitch from IMU [rad]
%   c_cfg   : Controller configuration (from controller_config.m)
%
% Outputs:
%   u     : Saturated control torque [N.m]
%   state : Updated controller state

if nargin < 4 || isempty(c_cfg)
    c_cfg = controller_config();
end

if isempty(state) || ~isfield(state, 'integrator')
    state.integrator = 0.0;
    state.last_u = 0.0;
end

Kp = c_cfg.pi.Kp;
Ki = c_cfg.pi.Ki;
u_sat = c_cfg.u_sat;
dt = c_cfg.dt;

e = theta_d - theta_m;

u_p = Kp * e;
int_candidate = state.integrator + e * dt;
u_i_candidate = Ki * int_candidate;
u_unsat = u_p + u_i_candidate;

u = max(-u_sat, min(u_sat, u_unsat));

% Anti-windup clamping
if c_cfg.pi.anti_windup
    is_sat_high = (u_unsat > u_sat) && (e > 0);
    is_sat_low  = (u_unsat < -u_sat) && (e < 0);
    if ~(is_sat_high || is_sat_low)
        state.integrator = int_candidate;
    end
else
    state.integrator = int_candidate;
end

state.last_u = u;
state.error = e;
end
