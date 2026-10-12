function params = get_quadruped_geometry()
% GET_QUADRUPED_GEOMETRY Authoritative physical & kinematic parameters of the quadruped.
%
% Aligned with Phase 2 - Robot Geometry foundation specification.

% Preserved SISO Pitch Control Parameters
params.J = 0.05;  % Body pitch moment of inertia [kg.m^2]
params.B = 0.10;  % Pitch damping coefficient [N.m.s/rad]
params.K = 0.00;  % Restoring stiffness [N.m/rad]

% Body Dimensions (Phase 2 Spec)
params.body.length = 0.40;      % m
params.body.width  = 0.20;      % m
params.body.height = 0.12;      % m

% Leg Dimensions (Phase 2 Spec)
params.leg.hipToKnee = 0.12;    % m
params.leg.kneeToFoot = 0.15;   % m

% Nominal Standing Height
params.standingHeight = 0.22;   % m

% Coordinate Frame Convention: x = forward, y = left, z = up
params.frame.x = "forward";
params.frame.y = "left";
params.frame.z = "up";

% Hip Positions (LF, RF, LH, RH)
L = params.body.length / 2;
W = params.body.width / 2;
params.hipPosition = [
     L,  W, 0;    % LF (Left Front)
     L, -W, 0;    % RF (Right Front)
    -L,  W, 0;    % LH (Left Hind)
    -L, -W, 0     % RH (Right Hind)
];

% Nominal Foot Positions
footX = 0.15;
footY = 0.10;
footZ = -params.standingHeight;
params.nominalFootPosition = [
     footX,  footY, footZ;     % LF
     footX, -footY, footZ;     % RF
    -footX,  footY, footZ;     % LH
    -footX, -footY, footZ      % RH
];
end
