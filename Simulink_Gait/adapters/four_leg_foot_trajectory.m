function foot_positions = four_leg_foot_trajectory(leg_phases, foot0, stepLength, stepHeight)
% FOUR_LEG_FOOT_TRAJECTORY Computes instantaneous 3D foot positions for all 4 legs.
%
% Inputs:
%   leg_phases : Normalized phases [4 x 1] in [0, 1) for [LF, RF, LH, RH]
%   foot0      : Nominal stance foot coordinates [4 x 3] in body/hip frame
%   stepLength : Stride length [m] (scalar)
%   stepHeight : Peak clearance swing height [m] (scalar)
%
% Outputs:
%   foot_positions : Desired 3D foot coordinates [4 x 3]

if nargin < 3 || isempty(stepLength), stepLength = 0.10; end
if nargin < 4 || isempty(stepHeight), stepHeight = 0.04; end

foot_positions = zeros(4, 3);

for leg = 1:4
    p = leg_phases(leg);
    x0 = foot0(leg, 1);
    y0 = foot0(leg, 2);
    z0 = foot0(leg, 3);
    
    if p < 0.5
        % Swing Phase (0 <= p < 0.5): normalized swing progression s in [0, 1]
        s = p / 0.5;
        dx = stepLength * (s - sin(2.0*pi*s)/(2.0*pi)) - (stepLength / 2.0);
        dz = stepHeight * (1.0 - cos(2.0*pi*s)) / 2.0;
    else
        % Stance Phase (0.5 <= p < 1.0): foot moves rearward relative to body
        s_st = (p - 0.5) / 0.5;
        dx = (stepLength / 2.0) - stepLength * s_st;
        dz = 0.0;
    end
    
    foot_positions(leg, 1) = x0 + dx;
    foot_positions(leg, 2) = y0;
    foot_positions(leg, 3) = z0 + dz;
end
end
