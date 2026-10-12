function benchmark_results = run_benchmark_suite(controllers, test_suite)
% RUN_BENCHMARK_SUITE Executes Tests 1-4 across PI, LSTM, and TD3 controllers.
%
% Ensures exact identical test conditions, plant parameters, disturbance profiles,
% and simulation durations.
%
% Inputs:
%   controllers : struct containing .pi, .lstm, .td3 controller structs
%   test_suite  : struct containing .test1, .test2, .test3, .test4

if nargin < 2 || isempty(test_suite)
    test_suite = generate_test_scenarios();
end

test_keys = {'test1', 'test2', 'test3', 'test4'};
ctrl_names = {'PI', 'LSTM', 'TD3'};

benchmark_results = struct();

for t_idx = 1:length(test_keys)
    t_key = test_keys{t_idx};
    scenario = test_suite.(t_key);
    fprintf('\n------------------------------------------------------------\n');
    fprintf('Running %s: %s\n', upper(t_key), scenario.name);
    fprintf('------------------------------------------------------------\n');
    
    for c_idx = 1:length(ctrl_names)
        c_name = ctrl_names{c_idx};
        c_field = lower(c_name);
        
        if ~isfield(controllers, c_field)
            fprintf('   [SKIP] Controller %s not provided.\n', c_name);
            continue;
        end
        
        ctrl_instance = controllers.(c_field);
        
        % Run simulation with exact scenario and plant parameters
        sim_out = simulate_closed_loop(c_name, ctrl_instance, scenario, scenario.plant_params);
        
        % Extract metrics
        metrics = compute_benchmark_metrics(sim_out, t_key);
        
        benchmark_results.(t_key).(c_field).sim_out = sim_out;
        benchmark_results.(t_key).(c_field).metrics = metrics;
        
        fprintf('   %-5s -> RMSE: %.4f rad | Peak Dev: %.4f rad | Control Energy: %.4f | Inf: %.1f us\n', ...
            c_name, metrics.RMSE, metrics.Peak_Deviation, metrics.Control_Energy, metrics.Mean_Inference_us);
    end
end

fprintf('\nAll benchmark test scenarios executed successfully.\n');
end
