function foot_pos = generate_foot_trajectory(phases, r_cfg, g_cfg, step_len, step_ht)
% GENERATE_FOOT_TRAJECTORY Computes continuous 3D foot positions for all 4 legs.
%
% Implements smooth cycloidal swing lift and linear stance retraction:
%   - Swing (p < beta): Smooth cycloidal x-translation, sinusoidal z-lift
%   - Stance (p >= beta): Linear ground contact retraction
%
% Inputs:
%   phases   : [4 x 1] Normalized phase in [0, 1)
%   r_cfg    : Robot configuration struct (from robot_config.m)
%   g_cfg    : Gait configuration struct (from gait_config.m)
%   step_len : (Optional) Overridden stride length [m]
%   step_ht  : (Optional) Overridden stride clearance [m]
%
% Outputs:
%   foot_pos : [4 x 3] Foot coordinates [x, y, z] relative to hip origin

if nargin < 2 || isempty(r_cfg), r_cfg = robot_config(); end
if nargin < 3 || isempty(g_cfg), g_cfg = gait_config();  end
if nargin < 4 || isempty(step_len), step_len = g_cfg.stepLength; end
if nargin < 5 || isempty(step_ht),  step_ht  = g_cfg.stepHeight; end

beta = g_cfg.dutyFactor; % 0.50
foot_pos = zeros(4, 3);

for leg = 1:4
    p  = phases(leg);
    x0 = r_cfg.nominalFootPosition(leg, 1);
    y0 = r_cfg.nominalFootPosition(leg, 2);
    z0 = r_cfg.nominalFootPosition(leg, 3);
    
    if p < (1.0 - beta)
        % Swing Phase (cycloidal lift)
        s = p / (1.0 - beta);
        dx = step_len * (s - sin(2*pi*s)/(2*pi)) - (step_len / 2.0);
        dz = step_ht * (1.0 - cos(2*pi*s)) / 2.0;
    else
        % Stance Phase (linear retraction against ground)
        s_st = (p - (1.0 - beta)) / beta;
        dx = (step_len / 2.0) - step_len * s_st;
        dz = 0.0;
    end
    
    foot_pos(leg, 1) = x0 + dx;
    foot_pos(leg, 2) = y0;
    foot_pos(leg, 3) = z0 + dz;
end
end
