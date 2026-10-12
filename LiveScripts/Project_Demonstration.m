%% QUADRUPED ROBOT GAIT AND BALANCE CONTROL: MASTER DEMONSTRATION
% This Live Script demonstrates the modular architecture, gait generation,
% inverse kinematics, balance control, and 3D simulation.

%% 1. Initialize Project & Load Central Configurations
clear; clc; close all;
proj_root = fileparts(mfilename('fullpath'));
if isempty(proj_root), proj_root = pwd; end
addpath(fullfile(proj_root, 'Shared', 'Configuration'));
addpath(fullfile(proj_root, 'MATLAB', 'Core', 'Gait'));
addpath(fullfile(proj_root, 'MATLAB', 'Core', 'Kinematics'));
addpath(fullfile(proj_root, 'MATLAB', 'Core', 'Balance'));
addpath(fullfile(proj_root, 'Simulink'));

r_cfg = robot_config();
g_cfg = gait_config();
c_cfg = controller_config();
s_cfg = simulation_config();

disp('Central configurations loaded successfully:');
disp(['Robot Mass: ', num2str(r_cfg.body.mass), ' kg']);
disp(['Gait Frequency: ', num2str(g_cfg.frequency), ' Hz']);
disp(['PI Gains: Kp = ', num2str(c_cfg.pi.Kp), ', Ki = ', num2str(c_cfg.pi.Ki)]);

%% 2. Generate 4-Leg Trot Gait Trajectories
t_vec = linspace(0, g_cfg.period, 100);
foot_traj = zeros(length(t_vec), 4, 3);
phases = zeros(length(t_vec), 4);

for k = 1:length(t_vec)
    [p, ~, ~] = compute_cpg_phases(t_vec(k), g_cfg);
    phases(k, :) = p';
    foot_traj(k, :, :) = generate_foot_trajectory(p, r_cfg, g_cfg);
end

figure('Name', 'Gait Trajectories');
subplot(2, 1, 1);
plot(t_vec, phases, 'LineWidth', 1.5);
grid on; ylabel('Phase p \in [0, 1)');
title('Coordinated 4-Leg Trot CPG Phases');
legend('LF', 'RF', 'LH', 'RH');

subplot(2, 1, 2);
plot(t_vec, squeeze(foot_traj(:, 1, 1)), 'b-', 'LineWidth', 1.5); hold on;
plot(t_vec, squeeze(foot_traj(:, 1, 3)), 'r-', 'LineWidth', 1.5);
grid on; xlabel('Time (s)'); ylabel('Foot Position (m)');
title('LF Foot Stride Trajectory (X: blue, Z: red)');
legend('X (stride)', 'Z (clearance)');

%% 3. Solve 12-DOF Analytical Inverse Kinematics
q_cmd = zeros(length(t_vec), 12);
for k = 1:length(t_vec)
    feet_k = squeeze(foot_traj(k, :, :));
    [q_k, feasible] = solve_inverse_kinematics(feet_k, r_cfg);
    q_cmd(k, :) = q_k';
end

figure('Name', 'Joint Angle Excursions');
plot(t_vec, rad2deg(q_cmd(:, 1:3)), 'LineWidth', 1.5);
grid on; xlabel('Time (s)'); ylabel('Joint Angle (deg)');
title('Left-Front Commanded Joint Angles (q1, q2, q3)');
legend('q1 (roll)', 'q2 (hip pitch)', 'q3 (knee pitch)');

%% 4. Run Baseline PI Pitch Stabilization Benchmark
res_pid = run_pid_case();
disp(['Baseline PI RMSE: ', num2str(res_pid.metrics.RMSE_rad), ' rad']);
disp(['Peak Disturbance Deviation: ', num2str(res_pid.metrics.Peak_Disturbance_deg), ' deg']);
