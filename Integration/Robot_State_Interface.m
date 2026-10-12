function balance_input = Robot_State_Interface(robotState, reference_pitch)
% ROBOT_STATE_INTERFACE Extracts balance control inputs from robot telemetry.
%
% Preserves the exact SISO controller interface definition:
%   Reference : theta_d(t) = desired body pitch (default = 0)
%   Output    : theta(t)   = actual body pitch from robotState
%   Error     : e(t)       = theta_d(t) - theta(t)

if nargin < 2 || isempty(reference_pitch)
    reference_pitch = 0.0;
end

balance_input.theta_d = reference_pitch;
balance_input.theta = robotState.theta;
balance_input.theta_dot = robotState.theta_dot;
balance_input.e = balance_input.theta_d - balance_input.theta;
balance_input.contact_states = robotState.contact_actual;
end
