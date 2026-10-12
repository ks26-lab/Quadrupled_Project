function gaitAdaptation = Balance_Gait_Interface(balanceState, robotState)
% BALANCE_GAIT_INTERFACE Evaluates balance telemetry and provides adaptation feedback to Gait.
%
% Outputs:
%   gaitAdaptation.step_height_mod  : Multiplier for step height
%   gaitAdaptation.step_length_mod  : Multiplier for step length
%   gaitAdaptation.gait_frequency_mod : Multiplier for gait cadence
%   gaitAdaptation.recovery_mode    : Boolean flag indicating disturbance recovery state

if nargin < 1 || isempty(balanceState)
    balanceState.e = 0.0;
    balanceState.u_pitch = 0.0;
end

pitch_error = abs(balanceState.e);

% Define thresholds based on Phase 4.6 and Phase 4.7 disturbance thresholds
E_MILD = 0.05;  % ~2.8 deg
E_SEVERE = 0.15;% ~8.6 deg

if pitch_error > E_SEVERE
    gaitAdaptation.step_height_mod = 1.25;     % Lift feet higher for obstacle/terrain clearance
    gaitAdaptation.step_length_mod = 0.70;     % Shorten stride for stability
    gaitAdaptation.gait_frequency_mod = 1.20;  % Increase stepping rate
    gaitAdaptation.recovery_mode = true;
elseif pitch_error > E_MILD
    gaitAdaptation.step_height_mod = 1.10;
    gaitAdaptation.step_length_mod = 0.85;
    gaitAdaptation.gait_frequency_mod = 1.10;
    gaitAdaptation.recovery_mode = false;
else
    gaitAdaptation.step_height_mod = 1.00;
    gaitAdaptation.step_length_mod = 1.00;
    gaitAdaptation.gait_frequency_mod = 1.00;
    gaitAdaptation.recovery_mode = false;
end
end
