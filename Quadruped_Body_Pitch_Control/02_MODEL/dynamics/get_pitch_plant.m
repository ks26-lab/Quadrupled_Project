function [G, sys_ss, s] = get_pitch_plant(params)
% GET_PITCH_PLANT Functional getter for pitch transfer function and state-space model.
%
% Usage:
%   [G, sys_ss, s] = get_pitch_plant()
%   [G, sys_ss, s] = get_pitch_plant(params)

if nargin < 1
    params = get_robot_parameters();
end

J = params.J;
B = params.B;
K = params.K;

s = tf('s');
G = 1 / (J*s^2 + B*s + K);

A = [0 1; 0 -B/J];
B_matrix = [0; 1/J];
C_matrix = [1 0];
D_matrix = 0;

sys_ss = ss(A, B_matrix, C_matrix, D_matrix);

end
