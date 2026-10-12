function simEngine = init_3d_simulation(cfg)
% INIT_3D_SIMULATION Initializes the 3D Quadruped Simulation Engine.
%
% Sets up:
%   1. Authoritative robot kinematic structure (L1=0.12m, L2=0.15m, Hips)
%   2. Conveyor belt platform with adjustable speed and dimensions
%   3. Configurable obstacle course (steps, ramps, blocks, uneven ground)
%   4. Integration bridge to Phase 1-8 gait & balance controllers
%   5. Interactive dashboard state & telemetry buffers
%
% Usage:
%   simEngine = init_3d_simulation()
%   simEngine = init_3d_simulation(cfg)

if nargin < 1 || isempty(cfg), cfg = struct(); end

sim_dir = fileparts(mfilename('fullpath'));
proj_root = fileparts(sim_dir);

% Ensure project paths
addpath(sim_dir);
addpath(fullfile(proj_root, '01_SETUP'));
addpath(fullfile(proj_root, '02_MODEL', 'parameters'));
addpath(fullfile(proj_root, 'Robot_Movement', 'Robot_Parameters'));
addpath(fullfile(proj_root, 'Gait_Movement'));
addpath(fullfile(proj_root, 'Gait_Movement', 'Phase_3_Inverse_Kinematics'));
addpath(fullfile(proj_root, 'Simulink_Gait', 'adapters'));
addpath(fullfile(proj_root, 'Integration'));

% 1. Authoritative Robot Geometry
geom = get_quadruped_geometry();
robot.body_length = geom.body.length; % 0.40 m
robot.body_width  = geom.body.width;  % 0.20 m
robot.body_height = geom.body.height; % 0.12 m
robot.L1 = geom.leg.hipToKnee;        % 0.12 m
robot.L2 = geom.leg.kneeToFoot;       % 0.15 m
robot.standing_height = geom.standingHeight; % 0.22 m
robot.hip_positions = geom.hipPosition;      % [4 x 3] (LF, RF, LH, RH)
robot.nominal_feet  = geom.nominalFootPosition;
robot.mass = 5.0; % kg
robot.J_pitch = geom.J; % 0.05 kg.m^2
robot.B_pitch = geom.B; % 0.10 N.m.s/rad
robot.Kp = 0.50;
robot.Ki = 0.20;
robot.u_sat = 0.40;
robot.k_torque_to_q = 0.05;

% 2. Conveyor Belt Configuration
conveyor.length = 4.0;  % m
conveyor.width  = 1.0;  % m
conveyor.height = 0.20; % m (top surface at z = 0.20 m)
conveyor.speed  = 0.10; % m/s (default forward)
conveyor.running = true;
conveyor.direction = 1.0; % +1 forward, -1 reverse
conveyor.friction_mu = 0.8;
conveyor.roller_radius = 0.05;
conveyor.texture_offset = 0.0;

% 3. Obstacle Course Configuration
obstacles = struct([]);

% Obstacle 1: Low Stepping Platform
obstacles(1).id = 1;
obstacles(1).type = 'step';
obstacles(1).name = 'Low Step Platform';
obstacles(1).pos = [0.80, 0.0, 0.20 + 0.015]; % [x, y, z] center
obstacles(1).size = [0.25, 0.60, 0.03];       % [dx, dy, dz]
obstacles(1).color = [0.9, 0.55, 0.1];
obstacles(1).active = true;

% Obstacle 2: Inclined Ramp
obstacles(2).id = 2;
obstacles(2).type = 'ramp';
obstacles(2).name = 'Inclined Ascending Ramp';
obstacles(2).pos = [1.40, 0.0, 0.20 + 0.02];
obstacles(2).size = [0.40, 0.60, 0.04];
obstacles(2).color = [0.2, 0.8, 0.3];
obstacles(2).active = true;

% Obstacle 3: Rectangular High Block
obstacles(3).id = 3;
obstacles(3).type = 'block';
obstacles(3).name = 'High Clearance Block';
obstacles(3).pos = [2.10, 0.0, 0.20 + 0.025];
obstacles(3).size = [0.15, 0.50, 0.05];
obstacles(3).color = [0.9, 0.2, 0.2];
obstacles(3).active = true;

% Obstacle 4: Missing Support Gap / Low Friction Ice Patch
obstacles(4).id = 4;
obstacles(4).type = 'patch';
obstacles(4).name = 'Low Friction Hazard Zone';
obstacles(4).pos = [2.70, 0.0, 0.20 + 0.002];
obstacles(4).size = [0.35, 0.70, 0.005];
obstacles(4).color = [0.2, 0.6, 0.9];
obstacles(4).active = true;

% 4. Gait & Controller State
gait.frequency   = 1.0;  % Hz
gait.step_length = 0.10; % m
gait.step_height = 0.04; % m
gait.phase_offsets = [0.0; 0.5; 0.5; 0.0]; % Trot
gait.vx_cmd = 0.10;
gait.vy_cmd = 0.00;
gait.wz_cmd = 0.00;

% 5. Simulation Horizon & Timing
time.dt = 0.005; % 200 Hz
time.current = 0.0;
time.is_paused = false;
time.speed_factor = 1.0;

% 6. Initial Robot World Pose
state.pos = [0.20; 0.0; conveyor.height + robot.standing_height]; % [x; y; z]
state.rpy = [0.0; deg2rad(0.0); 0.0]; % [roll; pitch; yaw]
state.vel = [0.0; 0.0; 0.0];
state.omega = [0.0; 0.0; 0.0];
state.integrator = 0.0;
state.q_cmd = zeros(12, 1);
state.foot_pos_world = zeros(4, 3);
state.contact_states = false(4, 1);
state.contact_loss_leg = 0; % 0: None, 1..4: Fault leg
state.external_force = [0; 0; 0]; % Disturbance force

% Assemble Engine Struct
simEngine.robot = robot;
simEngine.conveyor = conveyor;
simEngine.obstacles = obstacles;
simEngine.gait = gait;
simEngine.time = time;
simEngine.state = state;
simEngine.history = struct();
simEngine.history.time = [];
simEngine.history.theta = [];
simEngine.history.u = [];
simEngine.history.q = [];
simEngine.history.conveyor_x = [];

fprintf('[3D Simulation Engine] Initialized successfully.\n');
end
