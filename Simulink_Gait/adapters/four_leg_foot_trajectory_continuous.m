function [foot_positions, foot_velocities, foot_accelerations] = four_leg_foot_trajectory_continuous(leg_phases, foot0, stepLength, stepHeight, frequency, dutyFactor)
% FOUR_LEG_FOOT_TRAJECTORY_CONTINUOUS Computes continuous 3D foot position,
% velocity, and acceleration profiles with C1 transition continuity.
%
% Resolving Terminology:
%   - Stride Length (L_stride): Complete forward translation per full cycle T.
%   - Step Length (L_step): Distance between successive contralateral foot placements.
%     For symmetric trot (duty factor = 0.50): L_stride = stepLength.
%
% Swing Phase: 0 <= p < (1 - beta)
%   Normalized time: s = p / (1 - beta) in [0, 1]
%   x(s) = x0 - L_stride/2 + L_stride * (s - 0.5 * sin(2*pi*s) / pi)  [Cycloid / Smooth S-curve]
%   z(s) = z0 + stepHeight * sin(pi * s)
%
% Stance Phase: (1 - beta) <= p < 1.0
%   Normalized time: s_st = (p - (1 - beta)) / beta in [0, 1]
%   x(s_st) = x0 + L_stride/2 - L_stride * s_st  [Linear retract with velocity -v_body]
%   z(s_st) = z0                                 [Ground contact plane]
%
% Inputs:
%   leg_phases : [4 x 1] vector in [0, 1)
%   foot0      : [4 x 3] nominal foot root coordinates
%   stepLength : Stride length [m] (L_stride)
%   stepHeight : Clearance height [m]
%   frequency  : Gait cycle rate [Hz]
%   dutyFactor : Duty factor beta (default 0.50)

if nargin < 3 || isempty(stepLength), stepLength = 0.10; end
if nargin < 4 || isempty(stepHeight), stepHeight = 0.04; end
if nargin < 5 || isempty(frequency),  frequency  = 1.0; end
if nargin < 6 || isempty(dutyFactor), dutyFactor = 0.50; end

T = 1.0 / max(0.01, frequency);
T_sw = (1.0 - dutyFactor) * T;
T_st = dutyFactor * T;

foot_positions     = zeros(4, 3);
foot_velocities    = zeros(4, 3);
foot_accelerations = zeros(4, 3);

for leg = 1:4
    p = leg_phases(leg);
    x0 = foot0(leg, 1);
    y0 = foot0(leg, 2);
    z0 = foot0(leg, 3);
    
    if p < (1.0 - dutyFactor)
        % SWING PHASE
        s = p / (1.0 - dutyFactor);
        
        % Smooth cycloidal trajectory in x to eliminate velocity shock at touchdown
        % s in [0, 1]: x moves from (x0 - L/2) to (x0 + L/2)
        dx = stepLength * (s - sin(2 * pi * s) / (2 * pi)) - (stepLength / 2);
        dz = stepHeight * sin(pi * s);
        
        % Velocities
        vx = (stepLength / T_sw) * (1.0 - cos(2 * pi * s));
        vz = (stepHeight * pi / T_sw) * cos(pi * s);
        
        % Accelerations
        ax = (stepLength * 2 * pi / (T_sw^2)) * sin(2 * pi * s);
        az = -(stepHeight * (pi / T_sw)^2) * sin(pi * s);
    else
        % STANCE PHASE
        s_st = (p - (1.0 - dutyFactor)) / dutyFactor;
        
        % Foot translates rearward from (x0 + L/2) to (x0 - L/2)
        dx = (stepLength / 2) - stepLength * s_st;
        dz = 0.0;
        
        vx = -stepLength / T_st;
        vz = 0.0;
        
        ax = 0.0;
        az = 0.0;
    end
    
    foot_positions(leg, :)     = [x0 + dx, y0, z0 + dz];
    foot_velocities(leg, :)    = [vx, 0.0, vz];
    foot_accelerations(leg, :) = [ax, 0.0, az];
end
end
