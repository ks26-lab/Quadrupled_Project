function run_phase_3()
% RUN_PHASE_3 Standalone launcher for Phase 3: P Feedback Controller Design & Analysis.
%
% Dynamically ensures project paths are loaded, initializes required plant
% dependency G if missing, and executes Phase 3.
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
fprintf('  RUNNING: Phase 3 - Feedback Controller Design (Proportional)\n');
fprintf('============================================================\n');

% Execute Phase 3
phase_3_p_controller;

fprintf('\n>>> Phase 3 completed successfully.\n\n');
end
