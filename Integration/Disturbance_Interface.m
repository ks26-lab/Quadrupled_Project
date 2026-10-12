function disturbanceState = Disturbance_Interface(t, type)
% DISTURBANCE_INTERFACE Generates pitch disturbance torque and terrain force profiles.
%
% Re-uses validated disturbance profiles from Phase 3, 4.5, and 4.6:
%   - transient_pulse : 0.1 N.m pulse from t=2.0s to t=2.5s
%   - sustained_step  : 0.1 N.m step starting at t=2.0s
%   - sinusoidal      : Periodic terrain disturbance

if nargin < 2 || isempty(type)
    type = 'transient_pulse';
end

disturbanceState.t = t;
disturbanceState.tau_dist = 0.0;
disturbanceState.contact_force = zeros(3, 4);

switch lower(type)
    case 'transient_pulse'
        if t >= 2.0 && t <= 2.5
            disturbanceState.tau_dist = 0.10; % N.m
        end
    case 'sustained_step'
        if t >= 2.0
            disturbanceState.tau_dist = 0.10; % N.m
        end
    case 'sinusoidal'
        disturbanceState.tau_dist = 0.05 * sin(2 * pi * 1.5 * t);
    otherwise
        disturbanceState.tau_dist = 0.0;
end
end
