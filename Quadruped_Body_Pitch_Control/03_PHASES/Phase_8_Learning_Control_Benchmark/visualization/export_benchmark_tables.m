function [all_tables, summary_csv_path] = export_benchmark_tables(benchmark_results, output_dir)
% EXPORT_BENCHMARK_TABLES Aggregates and exports formatted CSV tables of metrics.

if nargin < 2 || isempty(output_dir)
    paths = project_paths();
    output_dir = fullfile(paths.results, 'tables');
end
if ~exist(output_dir, 'dir'), mkdir(output_dir); end

test_keys = {'test1', 'test2', 'test3', 'test4'};
ctrl_fields = {'pi', 'lstm', 'td3'};
ctrl_labels = {'PI_Baseline', 'LSTM_Supervised', 'TD3_Reinforcement_Learning'};

metric_names = { ...
    'RMSE_rad', ...
    'IAE_rad_s', ...
    'Peak_Deviation_rad', ...
    'Overshoot_pct', ...
    'Settling_Time_s', ...
    'Control_Energy_N2m2s', ...
    'Peak_Torque_Nm', ...
    'RMS_Torque_Nm', ...
    'Recovery_Time_s', ...
    'Mean_Inference_us' ...
};

field_mapping = { ...
    'RMSE', ...
    'IAE', ...
    'Peak_Deviation', ...
    'Overshoot_pct', ...
    'Settling_Time_s', ...
    'Control_Energy', ...
    'Peak_Torque', ...
    'RMS_Torque', ...
    'Recovery_Time_s', ...
    'Mean_Inference_us' ...
};

all_tables = struct();
summary_rows = {};

for t_idx = 1:length(test_keys)
    t_key = test_keys{t_idx};
    
    col_ctrl = {};
    mat_metrics = zeros(length(ctrl_fields), length(metric_names));
    
    for c_idx = 1:length(ctrl_fields)
        c_field = ctrl_fields{c_idx};
        c_label = ctrl_labels{c_idx};
        col_ctrl{c_idx, 1} = c_label;
        
        m_struct = benchmark_results.(t_key).(c_field).metrics;
        for m_idx = 1:length(field_mapping)
            val = m_struct.(field_mapping{m_idx});
            mat_metrics(c_idx, m_idx) = val;
        end
        
        summary_rows(end+1, :) = [{upper(t_key), c_label}, num2cell(mat_metrics(c_idx, :))];
    end
    
    T_test = array2table(mat_metrics, 'VariableNames', metric_names);
    T_test = [table(col_ctrl, 'VariableNames', {'Controller'}), T_test];
    
    all_tables.(t_key) = T_test;
    csv_file = fullfile(output_dir, sprintf('phase8_%s_metrics.csv', t_key));
    writetable(T_test, csv_file);
    fprintf('Exported %s table to: %s\n', t_key, csv_file);
end

% Master consolidated summary table
var_names = [{'Test_Scenario', 'Controller'}, metric_names];
T_master = cell2table(summary_rows, 'VariableNames', var_names);
summary_csv_path = fullfile(output_dir, 'phase8_master_benchmark_summary.csv');
writetable(T_master, summary_csv_path);
fprintf('Exported Master Summary table to: %s\n', summary_csv_path);
end
