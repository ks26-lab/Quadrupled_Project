function run_phase_4()
% RUN_PHASE_4 Standalone launcher for Phase 4: PI Controller Design & Pareto Tuning.
%
% Dynamically ensures project paths are loaded, initializes required plant
% dependency G if missing, and executes Phase 4 (Phases 4.1 to 4.4).
% Can be invoked from any working directory in MATLAB.

% Locate setup folder relative to this file
this_dir = fileparts(mfilename('fullpath'));
project_root = fileparts(fileparts(this_dir));
setup_dir = fullfile(project_root, '01_SETUP');

if exist(fullfile(setup_dir, 'project_setup.m'), 'file')
    addpath(setup_dir);
    project_setup(true);
end

fprintf('\n============================================================\n');
fprintf('  RUNNING: Phase 4 - PI Controller Design & Pareto Tuning\n');
fprintf('============================================================\n');

% Execute Phase 4
phase_4_pi_controller;

fprintf('\n>>> Phase 4 completed successfully.\n\n');
end
