%% PHASE 2 - ROBOT GEOMETRY
% Establish basic quadruped dimensions and coordinate frames.
%
% This is a FOUNDATION model.
% Detailed CAD-based dimensions can replace these values later.

clear;
clc;

%% 1. Body dimensions

robot.body.length = 0.40;      % m
robot.body.width  = 0.20;      % m
robot.body.height = 0.12;      % m

%% 2. Leg dimensions

robot.leg.hipToKnee = 0.12;    % m
robot.leg.kneeToFoot = 0.15;   % m

%% 3. Nominal standing height

robot.standingHeight = 0.22;   % m

%% 4. Coordinate convention
%
% x = forward
% y = left
% z = upward
%
% Body frame origin is at the approximate body center.

robot.frame.x = "forward";
robot.frame.y = "left";
robot.frame.z = "up";

%% 5. Hip locations
%
% Leg order:
% 1 = Left Front  (LF)
% 2 = Right Front (RF)
% 3 = Left Hind   (LH)
% 4 = Right Hind  (RH)

L = robot.body.length / 2;
W = robot.body.width / 2;

robot.hipPosition = [
     L,  W, 0;    % LF
     L, -W, 0;    % RF
    -L,  W, 0;    % LH
    -L, -W, 0     % RH
];

%% 6. Nominal foot positions

footX = 0.15;
footY = 0.10;
footZ = -robot.standingHeight;

robot.nominalFootPosition = [
     footX,  footY, footZ;     % LF
     footX, -footY, footZ;     % RF
    -footX,  footY, footZ;     % LH
    -footX, -footY, footZ      % RH
];

%% 7. Display

disp("Quadruped geometry initialized.");

disp("Hip positions:");
disp(robot.hipPosition);

disp("Nominal foot positions:");
disp(robot.nominalFootPosition);

%% 8. Simple visualization

if canUseGPU() || isempty(getenv('MATLAB_NO_DISPLAY'))
    try
        figure;
        hold on;
        grid on;
        axis equal;

        plot3( ...
            robot.hipPosition(:,1), ...
            robot.hipPosition(:,2), ...
            robot.hipPosition(:,3), ...
            'ko', 'MarkerFaceColor','k');

        plot3( ...
            robot.nominalFootPosition(:,1), ...
            robot.nominalFootPosition(:,2), ...
            robot.nominalFootPosition(:,3), ...
            'bo', 'MarkerFaceColor','b');

        xlabel('X (m)');
        ylabel('Y (m)');
        zlabel('Z (m)');
        title('Quadruped Coordinate Geometry');
    catch
        disp("Visualization figure skipped in headless mode.");
    end
end

%% 9. Save

current_dir = fileparts(mfilename('fullpath'));
if isempty(current_dir)
    current_dir = pwd;
end
save(fullfile(current_dir, "Phase2_RobotGeometry.mat"), "robot");

disp("Phase 2 complete.");
