function [q_command, delta_q_balance] = gait_balance_coupler(q_gait, u_pitch, k_torque_to_q)
% GAIT_BALANCE_COUPLER Combines nominal gait joint commands with balance correction.
%
% Reuses exact relationship from Integration/Gait_Balance_Interface.m:
%   q_command = q_gait + delta_q_balance
%
% Inputs:
%   q_gait         : Nominal 12-DOF joint commands [12 x 1] from gait generator
%   u_pitch        : SISO body pitch control torque [N.m] (scalar)
%   k_torque_to_q  : Torque to joint displacement mapping gain (default 0.05)
%
% Outputs:
%   q_command       : Net commanded 12-DOF joint vector [12 x 1]
%   delta_q_balance : Applied balance correction vector [12 x 1]

if nargin < 3 || isempty(k_torque_to_q), k_torque_to_q = 0.05; end

delta_q_balance = zeros(12, 1);

% Apply pitch correction symmetrically to hip pitch joints:
% Index 2  : LF thigh/hip pitch (+u_pitch)
% Index 5  : FR thigh/hip pitch (+u_pitch)
% Index 8  : RL thigh/hip pitch (-u_pitch)
% Index 11 : RR thigh/hip pitch (-u_pitch)
delta_q_balance(2)  =  k_torque_to_q * u_pitch;
delta_q_balance(5)  =  k_torque_to_q * u_pitch;
delta_q_balance(8)  = -k_torque_to_q * u_pitch;
delta_q_balance(11) = -k_torque_to_q * u_pitch;

q_command = q_gait + delta_q_balance;
end
