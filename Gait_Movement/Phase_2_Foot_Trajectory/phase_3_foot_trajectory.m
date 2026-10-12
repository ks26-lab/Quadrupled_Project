%% PHASE 3 - BASIC FOOT TRAJECTORY
% Generate a simple swing trajectory for one quadruped foot.
%
% Foundation only:
% Detailed gait trajectories will be developed later.

clear;
clc;

%% 1. Trajectory parameters

stepLength = 0.10;      % m
stepHeight = 0.04;      % m
T = 1.0;                % gait period
N = 100;

t = linspace(0,T,N);

%% 2. Initial foot position

x0 = 0.15;
y0 = 0.10;
z0 = -0.22;

%% 3. Forward foot movement

x = x0 + stepLength * (t/T);

y = y0 * ones(size(t));

%% 4. Swing height

% Simple sinusoidal lift.
z = z0 + stepHeight * sin(pi*t/T);

%% 5. Foot trajectory

footTrajectory = [x(:), y(:), z(:)];

%% 6. Plot trajectory

if canUseGPU() || isempty(getenv('MATLAB_NO_DISPLAY'))
    try
        figure;

        plot3(x,y,z,'LineWidth',2);
        grid on;
        axis equal;

        xlabel('X (m)');
        ylabel('Y (m)');
        zlabel('Z (m)');
        title('Basic Quadruped Foot Trajectory');
    catch
        disp("Visualization figure skipped in headless mode.");
    end
end

%% 7. Display

disp("Phase 3 foot trajectory generated.");

%% 8. Save

current_dir = fileparts(mfilename('fullpath'));
if isempty(current_dir)
    current_dir = pwd;
end
save(fullfile(current_dir, "Phase3_FootTrajectory.mat"), ...
     "t", "footTrajectory", ...
     "stepLength", "stepHeight");

disp("Phase 3 complete.");
