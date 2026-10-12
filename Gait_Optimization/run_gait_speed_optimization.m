function results = run_gait_speed_optimization()
% RUN_GAIT_SPEED_OPTIMIZATION Master executable launcher for gait speed optimization.
%
% Executes:
%   1. Parameter audit and physical limit retrieval
%   2. Analytical dynamics, torque, and power calculations
%   3. Closed-loop speed-safety simulation study across 7 candidates
%   4. Generation and export of CSV tables and comparative figures
%   5. Comprehensive engineering summary report

root_dir = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(root_dir, 'Gait_Optimization'));

study_results = run_speed_safety_study();
plot_speed_optimization(study_results);

fprintf('\n========================================================================================\n');
fprintf('  GAIT SPEED OPTIMIZATION COMPLETE: RESULTS EXPORTED TO Gait_Optimization/results/\n');
fprintf('========================================================================================\n\n');

results = study_results;
end
