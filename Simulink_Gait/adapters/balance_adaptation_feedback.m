function [step_length_eff, step_height_eff, recovery_mode] = balance_adaptation_feedback(e_pitch, base_length, base_height)
% BALANCE_ADAPTATION_FEEDBACK Evaluates pitch error and adapts stride parameters.
%
% Reuses verified thresholds from Integration/Balance_Gait_Interface.m:
%   - |e| > 0.15 rad : severe recovery (step_height * 1.25, step_length * 0.70)
%   - |e| > 0.05 rad : mild recovery   (step_height * 1.10, step_length * 0.85)
%   - otherwise      : nominal operation

err = abs(e_pitch);

if err > 0.15
    step_height_eff = base_height * 1.25;
    step_length_eff = base_length * 0.70;
    recovery_mode   = true;
elseif err > 0.05
    step_height_eff = base_height * 1.10;
    step_length_eff = base_length * 0.85;
    recovery_mode   = false;
else
    step_height_eff = base_height * 1.00;
    step_length_eff = base_length * 1.00;
    recovery_mode   = false;
end
end
