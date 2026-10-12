function [q, feasible] = solve_leg_ik(x, y, z, L1, L2)
% SOLVE_LEG_IK Analytical 3-DOF Leg Inverse Kinematics solver.
%
% Inputs:
%   x, y, z : Target foot coordinates in leg hip frame [m]
%   L1, L2  : Thigh and shank link lengths [m]
%
% Outputs:
%   q        : [3 x 1] joint angles [q1_roll; q2_pitch; q3_knee] in radians
%   feasible : Boolean flag indicating workspace reachability

if nargin < 4 || isempty(L1), L1 = 0.12; end
if nargin < 5 || isempty(L2), L2 = 0.15; end

q1 = atan2(y, -z);
r = sqrt(y^2 + z^2);
D = sqrt(x^2 + r^2);

cosQ3 = (D^2 - L1^2 - L2^2) / (2 * L1 * L2);

if abs(cosQ3) > 1.0
    feasible = false;
    q = zeros(3, 1);
    return;
end

feasible = true;
q3 = acos(cosQ3);
alpha = atan2(x, r);
beta = atan2(L2 * sin(q3), L1 + L2 * cos(q3));
q2 = alpha - beta;

q = [q1; q2; q3];
end
