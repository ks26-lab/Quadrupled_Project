function fig_handles = plot_comparative_responses(benchmark_results, output_fig_dir)
% PLOT_COMPARATIVE_RESPONSES Generates high-quality publication-ready comparison figures.

if nargin < 2 || isempty(output_fig_dir)
    paths = project_paths();
    output_fig_dir = fullfile(paths.results, 'figures');
end
if ~exist(output_fig_dir, 'dir'), mkdir(output_fig_dir); end

test_keys = {'test1', 'test2', 'test3', 'test4'};
test_titles = { ...
    'Test 1: Dynamic Reference Trajectory Tracking', ...
    'Test 2: Sustained Torque Disturbance Rejection (+0.15 N.m)', ...
    'Test 3: Contact Loss / Leg Slip Fault (-0.25 N.m Impact)', ...
    'Test 4: Plant Uncertainty & Model Mismatch (+35% J, -30% B)' ...
};

ctrl_fields = {'pi', 'lstm', 'td3'};
ctrl_names  = {'PI Baseline', 'LSTM Supervised', 'TD3 Actor-Critic'};
colors      = {'#1f77b4', '#2ca02c', '#d62728'}; % Blue, Green, Red

fig_handles = [];

for t_idx = 1:length(test_keys)
    t_key = test_keys{t_idx};
    
    f = figure('Name', sprintf('Phase 8 - %s', test_titles{t_idx}), ...
               'Position', [100 + t_idx*30, 100 + t_idx*20, 950, 700], ...
               'Visible', 'off');
    fig_handles(end+1) = f;
    
    % Subplot 1: Pitch Response vs Reference
    subplot(3, 1, 1);
    hold on;
    ref_time = benchmark_results.(t_key).pi.sim_out.time;
    ref_sig  = benchmark_results.(t_key).pi.sim_out.theta_d;
    plot(ref_time, ref_sig * (180/pi), 'k--', 'LineWidth', 1.8, 'DisplayName', '\theta_d (Reference)');
    
    for c_idx = 1:length(ctrl_fields)
        c_f = ctrl_fields{c_idx};
        out = benchmark_results.(t_key).(c_f).sim_out;
        plot(out.time, out.theta * (180/pi), 'Color', colors{c_idx}, 'LineWidth', 1.5, ...
             'DisplayName', sprintf('\\theta: %s', ctrl_names{c_idx}));
    end
    ylabel('Pitch \theta [deg]', 'FontWeight', 'bold');
    title(test_titles{t_idx}, 'FontSize', 11, 'FontWeight', 'bold');
    legend('Location', 'best', 'FontSize', 9);
    grid on;
    
    % Subplot 2: Tracking Error
    subplot(3, 1, 2);
    hold on;
    for c_idx = 1:length(ctrl_fields)
        c_f = ctrl_fields{c_idx};
        out = benchmark_results.(t_key).(c_f).sim_out;
        plot(out.time, out.error * (180/pi), 'Color', colors{c_idx}, 'LineWidth', 1.4, ...
             'DisplayName', sprintf('e: %s', ctrl_names{c_idx}));
    end
    ylabel('Pitch Error e [deg]', 'FontWeight', 'bold');
    legend('Location', 'best', 'FontSize', 9);
    grid on;
    
    % Subplot 3: Control Effort & Saturation Limits
    subplot(3, 1, 3);
    hold on;
    yline(0.40, 'k:', 'LineWidth', 1.2, 'DisplayName', 'Saturation \pm 0.4 N.m');
    yline(-0.40, 'k:', 'LineWidth', 1.2, 'HandleVisibility', 'off');
    
    for c_idx = 1:length(ctrl_fields)
        c_f = ctrl_fields{c_idx};
        out = benchmark_results.(t_key).(c_f).sim_out;
        plot(out.time, out.u, 'Color', colors{c_idx}, 'LineWidth', 1.4, ...
             'DisplayName', sprintf('u: %s', ctrl_names{c_idx}));
    end
    xlabel('Time [s]', 'FontWeight', 'bold');
    ylabel('Torque u [N.m]', 'FontWeight', 'bold');
    ylim([-0.50, 0.50]);
    legend('Location', 'best', 'FontSize', 9);
    grid on;
    
    % Save high-res PNG
    fig_file = fullfile(output_fig_dir, sprintf('phase8_%s_comparison.png', t_key));
    saveas(f, fig_file);
    fprintf('Saved Figure: %s\n', fig_file);
end

end
