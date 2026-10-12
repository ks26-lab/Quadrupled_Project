function legCommand = Gait_Balance_Interface(gaitState, balanceState)
% GAIT_BALANCE_INTERFACE Combines nominal gait joint commands with balance corrections.
%
% Enforces the strict architectural equation:
%   q_command = q_gait + delta_q_balance

if nargin < 2 || isempty(balanceState)
    balanceState.u_pitch = 0.0;
    balanceState.delta_q_balance = zeros(12, 1);
end

% Compute differential leg pitch adjustment from SISO balance torque
% Hip pitch indices in 12-DOF joint vector [FL_hip, FL_thigh, FL_knee, FR..., RL..., RR...]
delta_q = zeros(12, 1);
k_torque_to_q = 0.05; % Mapping gain from pitch torque to leg pitch differential

% Front legs pitch down, rear legs pitch up (or vice-versa) to compensate body pitch
delta_q(2) =  k_torque_to_q * balanceState.u_pitch; % FL thigh/hip pitch
delta_q(5) =  k_torque_to_q * balanceState.u_pitch; % FR thigh/hip pitch
delta_q(8) = -k_torque_to_q * balanceState.u_pitch; % RL thigh/hip pitch
delta_q(11)= -k_torque_to_q * balanceState.u_pitch; % RR thigh/hip pitch

legCommand.q_gait = gaitState.q_gait;
legCommand.delta_q_balance = delta_q + balanceState.delta_q_balance;
legCommand.q_command = legCommand.q_gait + legCommand.delta_q_balance;
end
