function [sys_tf, sys_ss] = get_pitch_dynamics_model(r_cfg)
% GET_PITCH_DYNAMICS_MODEL Functional getter for continuous SISO Body Pitch plant.
%
% Equations of motion:
%   J * theta_ddot + B * theta_dot + K * theta = tau_net
%
% Inputs:
%   r_cfg : Robot configuration struct (from robot_config.m)
%
% Outputs:
%   sys_tf : Continuous transfer function G(s) = 1 / (J*s^2 + B*s + K)
%   sys_ss : State-space model [A, B, C, D]

if nargin < 1 || isempty(r_cfg)
    r_cfg = robot_config();
end

J = r_cfg.dynamics.J; % 0.05
B = r_cfg.dynamics.B; % 0.10
K = r_cfg.dynamics.K; % 0.00

s = tf('s');
sys_tf = 1 / (J*s^2 + B*s + K);

A = [0, 1; -K/J, -B/J];
B_mat = [0; 1/J];
C = [1, 0];
D = 0;

sys_ss = ss(A, B_mat, C, D);
end
