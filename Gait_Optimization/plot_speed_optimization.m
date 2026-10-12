function fig_handles = plot_speed_optimization(study_results, save_dir)
% PLOT_SPEED_OPTIMIZATION Generates clear comparative engineering plots
% contrasting the Baseline Gait against the Proposed Faster Gait.

if nargin < 2 || isempty(save_dir)
    root_dir = fileparts(fileparts(mfilename('fullpath')));
    save_dir = fullfile(root_dir, 'Gait_Optimization', 'results');
end
if ~exist(save_dir, 'dir'), mkdir(save_dir); end

T = study_results.table;
params = study_results.params;

% Candidates to compare: Candidate 2 (Baseline 0.10 m/s) vs Candidate 4 (Proposed Optimal 0.21 m/s)
baseline = study_results.data{2};
proposed = study_results.data{4};

fig_handles = [];

%% Figure 1: Speed vs. Actuator Constraints and Feasibility Boundary
f1 = figure('Name', 'Gait Speed Optimization - Actuator Constraints', 'Position', [100, 100, 850, 600], 'Visible', 'off');
fig_handles(end+1) = f1;

subplot(2, 2, 1);
plot(T.Speed_mps, T.Max_Omega_rad_s, 'b-o', 'LineWidth', 2, 'MarkerFaceColor', 'b');
hold on;
yline(params.actuator.omega_max, 'r--', 'LineWidth', 1.8, 'DisplayName', 'Servo Limit \omega_{max} = 5.0 rad/s');
xlabel('Walking Speed [m/s]', 'FontWeight', 'bold');
ylabel('Peak Joint Velocity [rad/s]', 'FontWeight', 'bold');
title('Joint Velocity vs Walking Speed');
legend('Gait Candidates', 'Servo Limit', 'Location', 'northwest');
grid on;

subplot(2, 2, 2);
plot(T.Speed_mps, T.Max_Torque_Nm, 'm-s', 'LineWidth', 2, 'MarkerFaceColor', 'm');
hold on;
yline(params.actuator.tau_stall, 'r--', 'LineWidth', 1.8, 'DisplayName', 'Stall Torque = 2.5 N.m');
yline(params.actuator.tau_rated, 'k:', 'LineWidth', 1.5, 'DisplayName', 'Thermal Rated = 0.8 N.m');
xlabel('Walking Speed [m/s]', 'FontWeight', 'bold');
ylabel('Peak Joint Torque [N.m]', 'FontWeight', 'bold');
title('Joint Torque vs Walking Speed');
legend('Gait Candidates', 'Stall Limit', 'Thermal Rated', 'Location', 'northwest');
grid on;

subplot(2, 2, 3);
plot(T.Speed_mps, T.Avg_Mech_Power_W, 'g-d', 'LineWidth', 2, 'MarkerFaceColor', 'g');
hold on;
plot(T.Speed_mps, T.Avg_Bus_Current_A * 7.4, 'k--^', 'LineWidth', 1.8, 'MarkerFaceColor', 'k');
xlabel('Walking Speed [m/s]', 'FontWeight', 'bold');
ylabel('Power Demand [W]', 'FontWeight', 'bold');
title('Electrical & Mechanical Power');
legend('Mechanical Power', 'Total Electrical Power (7.4V)', 'Location', 'northwest');
grid on;

subplot(2, 2, 4);
plot(T.Speed_mps, T.Pitch_RMSE_deg, 'c-o', 'LineWidth', 2, 'MarkerFaceColor', 'c');
hold on;
yline(3.0, 'r--', 'LineWidth', 1.8, 'DisplayName', 'Stability Limit (3.0 deg)');
xlabel('Walking Speed [m/s]', 'FontWeight', 'bold');
ylabel('Pitch Tracking RMSE [deg]', 'FontWeight', 'bold');
title('Pitch Disturbance Deviation');
legend('Closed-Loop Pitch Error', 'Max Allowable Error', 'Location', 'northwest');
grid on;

saveas(f1, fullfile(save_dir, 'fig01_speed_actuator_constraints.png'));

%% Figure 2: Baseline (0.10 m/s) vs. Proposed Faster Gait (0.21 m/s)
f2 = figure('Name', 'Baseline vs Proposed Faster Gait', 'Position', [150, 150, 950, 650], 'Visible', 'off');
fig_handles(end+1) = f2;

% Joint velocity profiles over single gait cycle
subplot(3, 1, 1);
plot(baseline.dyn.t_vec, baseline.dyn.qdot_lf(:, 2), 'b-', 'LineWidth', 1.8, 'DisplayName', 'Baseline Hip Pitch (0.10 m/s)');
hold on;
plot(proposed.dyn.t_vec, proposed.dyn.qdot_lf(:, 2), 'r-', 'LineWidth', 1.8, 'DisplayName', 'Proposed Hip Pitch (0.21 m/s)');
plot(proposed.dyn.t_vec, proposed.dyn.qdot_lf(:, 3), 'r--', 'LineWidth', 1.5, 'DisplayName', 'Proposed Knee Pitch (0.21 m/s)');
yline(5.0, 'k:', 'LineWidth', 1.2, 'DisplayName', 'Actuator Limit \pm 5.0 rad/s');
yline(-5.0, 'k:', 'LineWidth', 1.2, 'HandleVisibility', 'off');
ylabel('Joint Rate \omega [rad/s]', 'FontWeight', 'bold');
title('Joint Velocity Comparison Over 1 Gait Cycle');
legend('Location', 'best');
grid on;

% Joint torque profiles over single gait cycle
subplot(3, 1, 2);
plot(baseline.dyn.t_vec, baseline.dyn.tau_lf(:, 2), 'b-', 'LineWidth', 1.8, 'DisplayName', 'Baseline Hip Torque (0.10 m/s)');
hold on;
plot(proposed.dyn.t_vec, proposed.dyn.tau_lf(:, 2), 'r-', 'LineWidth', 1.8, 'DisplayName', 'Proposed Hip Torque (0.21 m/s)');
plot(proposed.dyn.t_vec, proposed.dyn.tau_lf(:, 3), 'r--', 'LineWidth', 1.5, 'DisplayName', 'Proposed Knee Torque (0.21 m/s)');
yline(2.5, 'k:', 'LineWidth', 1.2, 'DisplayName', 'Stall Torque \pm 2.5 N.m');
ylabel('Joint Torque \tau [N.m]', 'FontWeight', 'bold');
title('Joint Torque Demand Over 1 Gait Cycle');
legend('Location', 'best');
grid on;

% Body pitch deviation in closed loop with PI balance control
subplot(3, 1, 3);
t_sim = linspace(0, 4.0, length(baseline.theta_hist));
plot(t_sim, baseline.theta_hist * (180/pi), 'b-', 'LineWidth', 1.6, 'DisplayName', 'Baseline Body Pitch (0.10 m/s)');
hold on;
plot(t_sim, proposed.theta_hist * (180/pi), 'r-', 'LineWidth', 1.6, 'DisplayName', 'Proposed Body Pitch (0.21 m/s)');
xlabel('Simulation Time [s]', 'FontWeight', 'bold');
ylabel('Pitch Angle \theta [deg]', 'FontWeight', 'bold');
title('Closed-Loop Pitch Stability Under Trot Disturbance & Transient Pulse');
legend('Location', 'best');
grid on;

saveas(f2, fullfile(save_dir, 'fig02_baseline_vs_proposed_gait.png'));

fprintf('Saved Optimization Figures to: %s\n', save_dir);
end
