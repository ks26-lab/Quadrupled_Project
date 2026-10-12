function [phases, stance, swing] = compute_cpg_phases(t, cfg)
% COMPUTE_CPG_PHASES Pure computational CPG phase generator for 4 legs.
%
% Inputs:
%   t   : Current simulation time (s)
%   cfg : Gait configuration struct (from gait_config.m)
%
% Outputs:
%   phases : [4 x 1] Normalized phase in [0, 1) for [LF; RF; LH; RH]
%   stance : [4 x 1] Boolean stance contact flags (1 = ground support)
%   swing  : [4 x 1] Boolean swing clearance flags (1 = leg in air)
%
% Reusable pure function with ZERO workspace side-effects.

if nargin < 2 || isempty(cfg)
    cfg = gait_config();
end

T = 1.0 / max(0.01, cfg.frequency);
global_phase = mod(t / T, 1.0);

phases = zeros(4, 1);
stance = false(4, 1);
swing  = false(4, 1);

for leg = 1:4
    p = mod(global_phase + cfg.phaseOffsets(leg), 1.0);
    phases(leg) = p;
    
    if p >= cfg.dutyFactor
        stance(leg) = true;
        swing(leg)  = false;
    else
        stance(leg) = false;
        swing(leg)  = true;
    end
end
end
