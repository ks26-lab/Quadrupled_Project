% PITCH_PLANT Script definition of quadruped body pitch plant.
%
% Generates:
%   s        : Laplace variable (tf('s'))
%   G        : Pitch transfer function 1/(J*s^2 + B*s + K)
%   A, B_matrix, C_matrix, D_matrix : State-space matrices
%   sys_ss   : State-space model

if ~exist('J', 'var') || ~exist('B', 'var') || ~exist('K', 'var')
    robot_parameters;
end

s = tf('s');
G = 1/(J*s^2 + B*s + K);

% State-space model
A = [0 1;
    0 -B/J];

B_matrix = [0;
    1/J];

C_matrix = [1 0];

D_matrix = 0;

sys_ss = ss(A, B_matrix, C_matrix, D_matrix);
