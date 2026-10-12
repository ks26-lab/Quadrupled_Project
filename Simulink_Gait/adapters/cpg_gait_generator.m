function [leg_phases, stance_states, swing_states] = cpg_gait_generator(t, frequency, phase_offsets)
% CPG_GAIT_GENERATOR Generates synchronized 4-leg normalized phase and contact states.
%
% Inputs:
%   t             : Current simulation time [s] (scalar)
%   frequency     : Gait cycle frequency [Hz] (scalar, default 1.0)
%   phase_offsets : Phase offsets [4 x 1] for [LF, RF, LH, RH]
%
% Outputs:
%   leg_phases    : Normalized phase in [0, 1) for each leg [4 x 1]
%   stance_states : Boolean contact flags [4 x 1] (1 = stance, 0 = swing)
%   swing_states  : Boolean swing flags [4 x 1] (1 = swing, 0 = stance)

if nargin < 2 || isempty(frequency), frequency = 1.0; end
if nargin < 3 || isempty(phase_offsets), phase_offsets = [0.0; 0.5; 0.5; 0.0]; end

T = 1.0 / max(0.01, frequency);
global_phase = mod(t / T, 1.0);

leg_phases = zeros(4, 1);
stance_states = false(4, 1);
swing_states = false(4, 1);

for leg = 1:4
    p = mod(global_phase + phase_offsets(leg), 1.0);
    leg_phases(leg) = p;
    
    % Trot gait: swing if p < 0.5, stance if p >= 0.5
    if p >= 0.5
        stance_states(leg) = true;
        swing_states(leg)  = false;
    else
        stance_states(leg) = false;
        swing_states(leg)  = true;
    end
end
end
