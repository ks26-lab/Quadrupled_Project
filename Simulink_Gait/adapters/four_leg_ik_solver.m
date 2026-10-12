function [q_12dof, feasible_flags] = four_leg_ik_solver(foot_positions, L1, L2)
% FOUR_LEG_IK_SOLVER Analytical 12-DOF Inverse Kinematics for all 4 legs.
%
% Inputs:
%   foot_positions : Desired foot coordinates [4 x 3] for [LF, RF, LH, RH]
%   L1, L2         : Thigh and shank link lengths [m]
%
% Outputs:
%   q_12dof        : Joint angles vector [12 x 1] in radians:
%                    [q1_LF, q2_LF, q3_LF, q1_RF, q2_RF, q3_RF,
%                     q1_LH, q2_LH, q3_LH, q1_RH, q2_RH, q3_RH]
%   feasible_flags : Boolean reachability flags [4 x 1]

if nargin < 2 || isempty(L1), L1 = 0.12; end
if nargin < 3 || isempty(L2), L2 = 0.15; end

q_12dof = zeros(12, 1);
feasible_flags = true(4, 1);

for leg = 1:4
    x = foot_positions(leg, 1);
    y = foot_positions(leg, 2);
    z = foot_positions(leg, 3);
    
    % Call authoritative Phase 3 IK formulation
    q1 = atan2(y, -z);
    r = sqrt(y^2 + z^2);
    D = sqrt(x^2 + r^2);
    
    cosQ3 = (D^2 - L1^2 - L2^2) / (2 * L1 * L2);
    
    if abs(cosQ3) > 1.0
        feasible_flags(leg) = false;
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
