function [u, ctrl] = pi_step(ctrl, theta_d, theta_m)
% PI_STEP Computes one discrete-time step for the PI baseline controller.
%
% Inputs:
%   ctrl    : controller state structure
%   theta_d : desired body pitch angle [rad]
%   theta_m : measured/actual body pitch angle [rad]
%
% Outputs:
%   u       : saturated control torque [N.m]
%   ctrl    : updated controller state

e = theta_d - theta_m;

% Proportional action
u_p = ctrl.Kp * e;

% Trial integrator update
int_cand = ctrl.integrator + e * ctrl.dt;
u_i_cand = ctrl.Ki * int_cand;
u_unsat = u_p + u_i_cand;

% Saturation
u = max(-ctrl.u_sat, min(ctrl.u_sat, u_unsat));

% Conditional anti-windup clamping:
% Only integrate if not saturated, or if the error pulls away from saturation
if ctrl.anti_windup
    is_saturated_high = (u_unsat > ctrl.u_sat) && (e > 0);
    is_saturated_low  = (u_unsat < -ctrl.u_sat) && (e < 0);
    if ~(is_saturated_high || is_saturated_low)
        ctrl.integrator = int_cand;
    end
else
    ctrl.integrator = int_cand;
end

ctrl.last_u = u;
end
