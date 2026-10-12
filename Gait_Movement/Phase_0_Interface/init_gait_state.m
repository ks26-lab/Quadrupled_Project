function [gaitState, interface] = init_gait_state(varargin)
% INIT_GAIT_STATE Initializes foundation interface data structures.
%
% Aligned with Phase 1 Gait Architecture & Interface standard.

p = inputParser;
addParameter(p, 'type', "STAND", @(x) ischar(x) || isstring(x));
addParameter(p, 'frequency', 1.0, @isnumeric);
addParameter(p, 'stepLength', 0.10, @isnumeric);
addParameter(p, 'stepHeight', 0.04, @isnumeric);
addParameter(p, 'bodyVelocity', [0 0 0], @isnumeric);
parse(p, varargin{:});

gaitState.type = string(p.Results.type);
gaitState.phase = 0.0;
gaitState.frequency = p.Results.frequency;
gaitState.stepLength = p.Results.stepLength;
gaitState.stepHeight = p.Results.stepHeight;
gaitState.bodyVelocity = p.Results.bodyVelocity;

% Robot State Interface
robotState.position = [0 0 0];
robotState.orientation = [0 0 0];
robotState.linearVelocity = [0 0 0];
robotState.angularVelocity = [0 0 0];
robotState.footPosition = zeros(4,3);
robotState.contact = ones(4,1);

% Balance State Interface
balanceState.pitch = 0;
balanceState.pitchRate = 0;
balanceState.pitchReference = 0;
balanceState.pitchError = 0;
balanceState.legCorrection = zeros(4,3);

% Leg Command Interface
legCommand.footPosition = zeros(4,3);
legCommand.jointPosition = zeros(4,3);

% Disturbance Interface
disturbanceState.pitchTorque = 0;
disturbanceState.terrainHeight = zeros(4,1);
disturbanceState.contactLoss = zeros(4,1);

% Combined Interface
interface.gait = gaitState;
interface.robot = robotState;
interface.balance = balanceState;
interface.command = legCommand;
interface.disturbance = disturbanceState;
end
