%% PHASE 4 - BASIC INVERSE KINEMATICS
% Simplified 3-DOF quadruped leg inverse kinematics.
%
% Foundation implementation.
% Detailed joint-axis definitions and robot-specific IK
% will be developed later.

clear;
clc;

%% 1. Leg dimensions

L1 = 0.12;      % Hip -> Knee
L2 = 0.15;      % Knee -> Foot

%% 2. Desired foot position

x = 0.15;
y = 0.10;
z = -0.22;

%% 3. Hip abduction/adduction angle

q1 = atan2(y,-z);

%% 4. Horizontal distance from hip joint

r = sqrt(y^2 + z^2);

%% 5. Distance from hip to foot

D = sqrt(x^2 + r^2);

%% 6. Workspace check

cosQ3 = (D^2 - L1^2 - L2^2) / ...
        (2*L1*L2);

if abs(cosQ3) > 1
    error("Desired foot position is outside the leg workspace.");
end

%% 7. Knee angle

q3 = acos(cosQ3);

%% 8. Hip pitch angle

alpha = atan2(x,r);

beta = atan2(L2*sin(q3), ...
             L1 + L2*cos(q3));

q2 = alpha - beta;

%% 9. Convert to degrees

jointAngles.rad = [q1 q2 q3];
jointAngles.deg = rad2deg(jointAngles.rad);

%% 10. Display

disp("Basic inverse kinematics result:");

fprintf("Hip angle  = %.2f deg\n", ...
        jointAngles.deg(1));

fprintf("Knee angle = %.2f deg\n", ...
        jointAngles.deg(2));

fprintf("Joint 3    = %.2f deg\n", ...
        jointAngles.deg(3));

%% 11. Save

current_dir = fileparts(mfilename('fullpath'));
if isempty(current_dir)
    current_dir = pwd;
end
save(fullfile(current_dir, "Phase4_IK_Result.mat"), ...
     "jointAngles", ...
     "x", "y", "z", ...
     "L1", "L2");

disp("Phase 4 complete.");
