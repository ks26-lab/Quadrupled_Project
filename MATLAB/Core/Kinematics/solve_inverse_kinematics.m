function [q_12dof, feasible] = solve_inverse_kinematics(foot_positions, r_cfg)
% SOLVE_INVERSE_KINEMATICS 12-DOF Analytical Inverse Kinematics for Quadruped.
%
% Solves closed-form joint angles [q1; q2; q3] for each leg:
%   q1: Hip roll abduction/adduction
%   q2: Hip pitch rotation
%   q3: Knee flexion/extension
%
% Inputs:
%   foot_positions : [4 x 3] Foot coordinates [x, y, z] in hip frame
%   r_cfg          : Robot configuration struct (from robot_config.m)
%
% Outputs:
%   q_12dof  : [12 x 1] Joint command vector in radians
%   feasible : [4 x 1] Boolean reachability flags (true if within workspace)

if nargin < 2 || isempty(r_cfg)
    r_cfg = robot_config();
end

L1 = r_cfg.leg.L1; % 0.12 m
L2 = r_cfg.leg.L2; % 0.15 m

q_12dof = zeros(12, 1);
feasible = true(4, 1);

for leg = 1:4
    x = foot_positions(leg, 1);
    y = foot_positions(leg, 2);
    z = foot_positions(leg, 3);
    
    q1 = atan2(y, -z);
    r = sqrt(y^2 + z^2);
    D = sqrt(x^2 + r^2);
    cosQ3 = (D^2 - L1^2 - L2^2) / (2.0 * L1 * L2);
    
    if abs(cosQ3) > 1.0
        feasible(leg) = false;
        q_leg = zeros(3, 1);
    else
        q3 = acos(cosQ3);
        alpha = atan2(x, r);
        beta = atan2(L2 * sin(q3), L1 + L2 * cos(q3));
        q2 = alpha - beta;
        q_leg = [q1; q2; q3];
    end
    
    base_idx = (leg - 1) * 3;
    q_12dof(base_idx + 1) = q_leg(1);
    q_12dof(base_idx + 2) = q_leg(2);
    q_12dof(base_idx + 3) = q_leg(3);
end
end
