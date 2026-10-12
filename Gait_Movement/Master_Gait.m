%% ================================================================
%  QUADRUPED ROBOT - MASTER GAIT FOUNDATION
%  Phases 5-8: Single Leg -> Gait -> Walking -> Motion Command
%
%  PURPOSE:
%  Foundation implementation only.
%
%  This file prepares the locomotion pipeline:
%
%  Motion Command
%       ↓
%  Gait Generator
%       ↓
%  Foot Trajectory
%       ↓
%  Inverse Kinematics
%       ↓
%  Joint Commands
%
%  Future closed-loop architecture:
%
%  Gait <-> Robot Movement <-> Balance Control
%
%  Existing Balance_Control code is NOT modified here.
% ================================================================

clear;
clc;
close all;

fprintf('\n============================================\n');
fprintf(' QUADRUPED GAIT FOUNDATION\n');
fprintf(' Phases 5-8\n');
fprintf('============================================\n\n');

%% ================================================================
% PHASE 5 - SINGLE LEG FOUNDATION
% ================================================================

fprintf('PHASE 5: Single-leg trajectory + IK\n');

%% Robot leg dimensions

L1 = 0.12;       % Hip -> Knee (m)
L2 = 0.15;       % Knee -> Foot (m)

%% Nominal foot position

x0 = 0.15;
y0 = 0.10;
z0 = -0.22;

%% Motion parameters

stepLength = 0.10;
stepHeight = 0.04;

T = 1.0;
N = 100;

t = linspace(0,T,N);

%% Foot trajectory

x = x0 + stepLength*(t/T);

y = y0*ones(size(t));

z = z0 + stepHeight*sin(pi*t/T);

footTrajectory = [x(:),y(:),z(:)];

%% Joint trajectory

q_single = zeros(N,3);

for k = 1:N

    r = sqrt(y(k)^2 + z(k)^2);
    D = sqrt(x(k)^2 + r^2);

    cosQ3 = ...
        (D^2-L1^2-L2^2)/(2*L1*L2);

    if abs(cosQ3) > 1
        error('Phase 5: Foot position outside IK workspace.');
    end

    q1 = atan2(y(k),-z(k));

    q3 = acos(cosQ3);

    alpha = atan2(x(k),r);

    beta = atan2( ...
        L2*sin(q3), ...
        L1 + L2*cos(q3));

    q2 = alpha-beta;

    q_single(k,:) = [q1 q2 q3];

end

fprintf('  Single-leg IK generated successfully.\n');

%% ================================================================
% PHASE 6 - FOUR LEG GAIT FOUNDATION
% ================================================================

fprintf('\nPHASE 6: Four-leg trot gait\n');

%% Leg order

% 1 = Left Front
% 2 = Right Front
% 3 = Left Hind
% 4 = Right Hind

legNames = ["LF","RF","LH","RH"];

%% Trot phase offsets

phaseOffset = [
    0.0;       % LF
    0.5;       % RF
    0.5;       % LH
    0.0        % RH
];

%% Simulation time

N_gait = 200;

t_gait = linspace(0,T,N_gait);

phase = zeros(N_gait,4);

stance = false(N_gait,4);
swing = false(N_gait,4);

for k = 1:N_gait

    for leg = 1:4

        phase(k,leg) = mod( ...
            t_gait(k)/T + phaseOffset(leg),1);

        stance(k,leg) = phase(k,leg) >= 0.5;

        swing(k,leg) = ~stance(k,leg);

    end

end

fprintf('  Trot gait phases generated.\n');

%% ================================================================
% PHASE 7 - BASIC QUADRUPED WALKING
% ================================================================

fprintf('\nPHASE 7: Four-leg walking trajectory\n');

%% Nominal foot positions

foot0 = [
     0.15,  0.10, -0.22;    % LF
     0.15, -0.10, -0.22;    % RF
    -0.15,  0.10, -0.22;    % LH
    -0.15, -0.10, -0.22     % RH
];

%% Allocate foot trajectory

footPosition = zeros(N_gait,4,3);

for k = 1:N_gait

    for leg = 1:4

        p = phase(k,leg);

        if p < 0.5

            % Swing

            s = p/0.5;

            dx = stepLength*s;

            dz = stepHeight*sin(pi*s);

        else

            % Stance

            s = (p-0.5)/0.5;

            dx = stepLength*(1-s);

            dz = 0;

        end

        footPosition(k,leg,1) = ...
            foot0(leg,1) + dx;

        footPosition(k,leg,2) = ...
            foot0(leg,2);

        footPosition(k,leg,3) = ...
            foot0(leg,3) + dz;

    end

end

fprintf('  Four-leg walking trajectory generated.\n');

%% ================================================================
% PHASE 8 - BASIC MOTION COMMAND INTERFACE
% ================================================================

fprintf('\nPHASE 8: Motion command interface\n');

%% User-level motion command

motion.vx = 0.20;       % Forward velocity (m/s)
motion.vy = 0.00;       % Lateral velocity (m/s)
motion.wz = 0.00;       % Yaw rate (rad/s)

%% Gait settings

gait.type = "TROT";
gait.frequency = 1.0;
gait.stepHeight = stepHeight;

%% Determine movement mode

if abs(motion.wz) > 1e-6

    motion.mode = "TURN";

elseif abs(motion.vx) > 1e-6 || ...
       abs(motion.vy) > 1e-6

    motion.mode = "WALK";

else

    motion.mode = "STAND";

end

%% Approximate velocity-step relationship

if gait.frequency > 0

    gait.stepLength = ...
        abs(motion.vx)/gait.frequency;

else

    gait.stepLength = 0;

end

fprintf('  Motion mode: %s\n',motion.mode);

%% ================================================================
% FUTURE BALANCE INTERFACE
% ================================================================
%
% These variables are intentionally placeholders.
%
% The existing SISO balance controller will later provide:
%
%       delta_q_balance
%
% The final command will become:
%
%       q_command = q_gait + delta_q_balance
%
% The robot will provide:
%
%       theta
%       theta_dot
%       contact states
%
% The balance system will provide:
%
%       pitch correction
%
% The gait system will eventually receive:
%
%       stability information
%       disturbance information
%
% ================================================================

balanceState.pitch = 0;
balanceState.pitchReference = 0;
balanceState.pitchError = 0;

delta_q_balance = zeros(4,3);

%% Nominal gait joint command placeholder

q_gait = zeros(4,3);

%% Final command interface

q_command = q_gait + delta_q_balance;

%% ================================================================
% VISUALIZATION
% ================================================================

if canUseGPU() || isempty(getenv('MATLAB_NO_DISPLAY'))
    try
        figure;

        plot3( ...
            squeeze(footPosition(:,1,1)), ...
            squeeze(footPosition(:,1,2)), ...
            squeeze(footPosition(:,1,3)), ...
            'LineWidth',2);

        grid on;
        axis equal;

        xlabel('X (m)');
        ylabel('Y (m)');
        zlabel('Z (m)');

        title('Quadruped LF Foot Trajectory');

        %% Gait phase plot

        figure;

        plot(t_gait,phase,'LineWidth',1.3);

        grid on;

        xlabel('Time (s)');
        ylabel('Normalized Phase');

        title('Four-Leg Trot Gait');

        legend(legNames);

        %% Single-leg joint plot

        figure;

        plot(t,rad2deg(q_single),'LineWidth',1.3);

        grid on;

        xlabel('Time (s)');
        ylabel('Joint Angle (deg)');

        title('Single-Leg Joint Trajectory');

        legend('q_1','q_2','q_3');
    catch
        disp('Visualization figures skipped in headless mode.');
    end
end

%% ================================================================
% OUTPUT STRUCTURES
% ================================================================

gaitState.type = gait.type;
gaitState.phase = phase;
gaitState.stance = stance;
gaitState.swing = swing;
gaitState.frequency = gait.frequency;
gaitState.stepLength = stepLength;
gaitState.stepHeight = stepHeight;

robotState.footPosition = footPosition;
robotState.nominalFootPosition = foot0;

movementCommand.velocity = [
    motion.vx;
    motion.vy;
    motion.wz
];

movementCommand.mode = motion.mode;

movementCommand.stepLength = gait.stepLength;
movementCommand.stepHeight = gait.stepHeight;
movementCommand.frequency = gait.frequency;

%% ================================================================
% SAVE FOUNDATION RESULTS
% ================================================================

current_dir = fileparts(mfilename('fullpath'));
if isempty(current_dir)
    current_dir = pwd;
end

save( ...
    fullfile(current_dir, 'Master_Gait_Foundation.mat'), ...
    'gaitState', ...
    'robotState', ...
    'movementCommand', ...
    'balanceState', ...
    'q_command', ...
    'q_single', ...
    'footTrajectory');

fprintf('\n============================================\n');
fprintf(' PHASES 5-8 FOUNDATION COMPLETE\n');
fprintf('============================================\n');

fprintf('\nCreated:\n');
fprintf('  Single-leg trajectory\n');
fprintf('  Single-leg IK\n');
fprintf('  Four-leg trot phase\n');
fprintf('  Four-leg foot trajectories\n');
fprintf('  Motion command interface\n');
fprintf('  Balance interface placeholders\n');

fprintf('\nCurrent architecture:\n');
fprintf('  Gait -> Foot -> IK -> Joint Command\n');

fprintf('\nFuture architecture:\n');
fprintf('  Gait <-> Robot <-> Balance\n');

fprintf('\nNo existing Balance_Control files were modified.\n');
