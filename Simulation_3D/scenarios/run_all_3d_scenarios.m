function results = run_all_3d_scenarios(save_dir)
% RUN_ALL_3D_SCENARIOS Programmatic batch execution of all 6 Demonstration Scenarios
% for the 3D Quadruped Conveyor & Obstacle Course environment.
%
% Scenarios:
%   1. Nominal Gait (Conveyor stopped, flat ground)
%   2. Moving Conveyor (Forward & Reverse belt velocities)
%   3. Obstacle Traversal (Step platform and rectangular block encounters)
%   4. Uneven Terrain & Ramp Climbing
%   5. Disturbance Rejection (External body force / pitch pulse)
%   6. Contact Loss Fault (Right-Front leg support loss, testing fault adaptation)
%
% Usage:
%   results = run_all_3d_scenarios()
%   results = run_all_3d_scenarios(save_dir)

if nargin < 1 || isempty(save_dir)
    save_dir = fullfile(fileparts(mfilename('fullpath')), 'logs');
end
if ~exist(save_dir, 'dir'), mkdir(save_dir); end

sim_dir = fileparts(mfilename('fullpath'));
addpath(fullfile(sim_dir, 'controllers'));

fprintf('\n========================================================================\n');
fprintf('  EXECUTING 6 DEMONSTRATION SCENARIOS: 3D CONVEYOR & OBSTACLE SIMULATION\n');
fprintf('========================================================================\n\n');

scenarios = {
    'Scenario 1: Nominal Gait', ...
    'Scenario 2: Moving Conveyor', ...
    'Scenario 3: Obstacle Traversal', ...
    'Scenario 4: Uneven Terrain Ramp', ...
    'Scenario 5: Disturbance Rejection', ...
    'Scenario 6: RF Contact Loss Fault'
};

results = struct();
summary_rows = cell(length(scenarios), 7);

for s = 1:length(scenarios)
    s_name = scenarios{s};
    fprintf('Running [%d/6]: %s ...\n', s, s_name);
    
    sim = init_3d_simulation();
    
    % Configure Scenario Parameters
    switch s
        case 1
            sim.conveyor.speed = 0.0;
        case 2
            sim.conveyor.speed = 0.12; % Forward belt
        case 3
            sim.conveyor.speed = 0.08;
            % Position obstacle in front of robot
            sim.obstacles(1).pos = [0.45, 0, 0.215];
        case 4
            sim.conveyor.speed = 0.06;
            sim.obstacles(2).pos = [0.40, 0, 0.22];
        case 5
            sim.conveyor.speed = 0.10;
        case 6
            sim.conveyor.speed = 0.08;
            sim.state.contact_loss_leg = 2; % RF leg disabled
    end
    
    % Simulate 4.0 seconds (800 steps of 5ms)
    steps = 800;
    theta_log = zeros(steps, 1);
    u_log     = zeros(steps, 1);
    x_log     = zeros(steps, 1);
    
    for k = 1:steps
        t = (k - 1) * 0.005;
        
        % In Scenario 5, inject disturbance pulse at t=2.0s
        if s == 5 && t >= 2.0 && t <= 2.5
            sim.state.external_force(1) = 2.0; % 2.0 N
        else
            sim.state.external_force(1) = 0.0;
        end
        
        sim = step_3d_simulation(sim);
        
        theta_log(k) = sim.state.rpy(2);
        u_log(k)     = sim.history.u(end);
        x_log(k)     = sim.state.pos(1);
    end
    
    % Metric extraction
    rmse_pitch = rad2deg(sqrt(mean(theta_log.^2)));
    mae_pitch  = rad2deg(mean(abs(theta_log)));
    max_pitch  = rad2deg(max(abs(theta_log)));
    max_u      = max(abs(u_log));
    dist_walked = x_log(end) - x_log(1);
    
    res.scenario = s_name;
    res.rmse_pitch_deg = rmse_pitch;
    res.mae_pitch_deg  = mae_pitch;
    res.max_pitch_deg  = max_pitch;
    res.max_torque_Nm  = max_u;
    res.distance_m     = dist_walked;
    res.theta_history  = theta_log;
    res.u_history      = u_log;
    
    results.(sprintf('scenario_%d', s)) = res;
    
    summary_rows{s, 1} = s_name;
    summary_rows{s, 2} = sim.conveyor.speed;
    summary_rows{s, 3} = max_pitch;
    summary_rows{s, 4} = rmse_pitch;
    summary_rows{s, 5} = mae_pitch;
    summary_rows{s, 6} = max_u;
    summary_rows{s, 7} = dist_walked;
    
    fprintf('   Completed: Max Pitch = %.2f deg | RMSE = %.3f deg | Max Torque = %.2f N.m\n', ...
        max_pitch, rmse_pitch, max_u);
end

% Summary Table
res_table = cell2table(summary_rows, 'VariableNames', ...
    {'Scenario', 'ConveyorSpeed_mps', 'MaxPitch_deg', 'RMSE_deg', 'MAE_deg', 'MaxTorque_Nm', 'DistanceWalked_m'});
results.summary_table = res_table;

fprintf('\n========================================================================\n');
fprintf('  DEMONSTRATION SCENARIOS QUANTITATIVE SUMMARY\n');
fprintf('========================================================================\n');
disp(res_table);

% Save MAT Results
mat_file = fullfile(save_dir, 'scenario_batch_results.mat');
save(mat_file, 'results', 'res_table');
fprintf('Results saved to: %s\n\n', mat_file);

% Generate Engineering Plots
if canUseGPU() || isempty(getenv('MATLAB_NO_DISPLAY'))
    try
        fig = figure('Name', '3D Conveyor Simulation Scenario Responses', 'Position', [100, 100, 1000, 700]);
        time_vec = (0:799) * 0.005;
        
        subplot(2, 1, 1);
        hold on; grid on;
        colors = {'b-', 'g-', 'r-', 'm-', 'c-', 'k--'};
        for s = 1:6
            f_name = sprintf('scenario_%d', s);
            plot(time_vec, rad2deg(results.(f_name).theta_history), colors{s}, 'LineWidth', 1.5);
        end
        yline(0, 'k:');
        xlabel('Time (s)'); ylabel('Pitch \theta (deg)');
        title('Comparative Pitch Responses across All 6 Scenarios');
        legend(scenarios, 'Location', 'Best');
        
        subplot(2, 1, 2);
        hold on; grid on;
        for s = 1:6
            f_name = sprintf('scenario_%d', s);
            plot(time_vec, results.(f_name).u_history, colors{s}, 'LineWidth', 1.5);
        end
        yline(0.40, 'k:'); yline(-0.40, 'k:');
        xlabel('Time (s)'); ylabel('Balance Torque u (N\cdotm)');
        title('SISO Pitch Control Effort Across Scenarios (\pm 0.40 N\cdotm Saturation)');
        legend(scenarios, 'Location', 'Best');
        
        plot_png = fullfile(fileparts(save_dir), 'plots', 'scenarios_comparison.png');
        if ~exist(fileparts(plot_png), 'dir'), mkdir(fileparts(plot_png)); end
        saveas(fig, plot_png);
        fprintf('Plot saved to: %s\n', plot_png);
    catch
        fprintf('Plot saving skipped in headless mode.\n');
    end
end
end
