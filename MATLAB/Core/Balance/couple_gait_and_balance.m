function [q_command, delta_q] = couple_gait_and_balance(q_gait, u_pitch, k_torque_to_q)
% COUPLE_GAIT_AND_BALANCE Integrates balance torque corrections into 12-DOF joint commands.
%
% Implements the core architectural relationship:
%   q_command = q_gait + delta_q_balance
%
% Symmetrically adjusts hip pitch joints:
%   Index 2  : LF thigh/hip pitch (+u_pitch)
%   Index 5  : FR thigh/hip pitch (+u_pitch)
%   Index 8  : LH thigh/hip pitch (-u_pitch)
%   Index 11 : RH thigh/hip pitch (-u_pitch)
%
% Inputs:
%   q_gait        : [12 x 1] Nominal joint command from gait IK
%   u_pitch       : Saturated balance torque [N.m]
%   k_torque_to_q : (Optional) Gain mapping torque to rad (default 0.05)
%
% Outputs:
%   q_command : [12 x 1] Commanded joint angles
%   delta_q   : [12 x 1] Applied balance offset

if nargin < 3 || isempty(k_torque_to_q)
    k_torque_to_q = 0.05;
end

delta_q = zeros(12, 1);
delta_q(2)  =  k_torque_to_q * u_pitch;
delta_q(5)  =  k_torque_to_q * u_pitch;
delta_q(8)  = -k_torque_to_q * u_pitch;
delta_q(11) = -k_torque_to_q * u_pitch;

q_command = q_gait + delta_q;
end
