function params = get_robot_parameters()
% GET_ROBOT_PARAMETERS Functional getter for quadruped pitch parameters.
%
% Returns a struct containing:
%   J : Body pitch moment of inertia [kg.m^2]
%   B : Pitch damping coefficient [N.m.s/rad]
%   K : Initial restoring stiffness [N.m/rad]

params.J = 0.05;      % Body pitch moment of inertia [kg.m^2]
params.B = 0.10;      % Damping coefficient [N.m.s/rad]
params.K = 0.00;      % Initial restoring stiffness [N.m/rad]
end
