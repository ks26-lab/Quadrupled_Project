%% PHASE 1 - GAIT ARCHITECTURE & INTERFACE
% Quadruped Robot Gait and Balance Control
%
% Purpose:
%   Establish the basic data interface between:
%   Gait Movement <-> Robot Movement <-> Balance Control
%
% This is FOUNDATION CODE.
% Detailed gait logic will be implemented in later phases.

clear;
clc;

%% 1. Basic gait configuration

gait.type = "STAND";
gait.phase = 0;
gait.frequency = 1.0;       % Hz
gait.stepLength = 0.10;     % m
gait.stepHeight = 0.04;     % m
gait.bodyVelocity = [0 0 0];

%% 2. Gait state

gaitState.type = gait.type;
gaitState.phase = gait.phase;
gaitState.frequency = gait.frequency;
gaitState.stepLength = gait.stepLength;
gaitState.stepHeight = gait.stepHeight;
gaitState.bodyVelocity = gait.bodyVelocity;

%% 3. Robot state interface

robotState.position = [0 0 0];       % [x y z]
robotState.orientation = [0 0 0];    % [roll pitch yaw]
robotState.linearVelocity = [0 0 0];
robotState.angularVelocity = [0 0 0];

% Foot positions: LF RF LH RH
robotState.footPosition = zeros(4,3);

% Contact state: 1 = contact, 0 = no contact
robotState.contact = ones(4,1);

%% 4. Balance interface

balanceState.pitch = 0;
balanceState.pitchRate = 0;
balanceState.pitchReference = 0;
balanceState.pitchError = 0;

% Future corrective command from balance controller
balanceState.legCorrection = zeros(4,3);

%% 5. Leg command interface

legCommand.footPosition = zeros(4,3);
legCommand.jointPosition = zeros(4,3);

%% 6. Disturbance interface

disturbanceState.pitchTorque = 0;
disturbanceState.terrainHeight = zeros(4,1);
disturbanceState.contactLoss = zeros(4,1);

%% 7. Combined interface

interface.gait = gaitState;
interface.robot = robotState;
interface.balance = balanceState;
interface.command = legCommand;
interface.disturbance = disturbanceState;

%% 8. Display

disp("Phase 1 interface initialized.");

disp("Gait -> Robot:");
disp(interface.gait);

disp("Robot -> Balance:");
disp(interface.robot);

disp("Balance -> Robot:");
disp(interface.balance);

disp("Balance -> Gait: available for future adaptation.");

%% 9. Save interface

% Determine current directory for saving Phase1_Interface.mat
current_dir = fileparts(mfilename('fullpath'));
if isempty(current_dir)
    current_dir = pwd;
end
save(fullfile(current_dir, "Phase1_Interface.mat"), "interface");

disp("Phase 1 complete.");
