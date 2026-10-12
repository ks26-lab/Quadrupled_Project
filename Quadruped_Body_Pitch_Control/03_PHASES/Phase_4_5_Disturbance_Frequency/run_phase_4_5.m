function run_phase_4_5()
% RUN_PHASE_4_5 Standalone launcher for Phase 4.5: Disturbance Rejection Validation.
%
% Dynamically ensures project paths are loaded, initializes required plant
% and frozen PI controller dependencies if missing, and executes Phase 4.5.
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
fprintf('  RUNNING: Phase 4.5 - Disturbance Rejection Validation\n');
fprintf('============================================================\n');

% Execute Phase 4.5
phase_4_5_disturbance;

fprintf('\n>>> Phase 4.5 completed successfully.\n\n');
end
