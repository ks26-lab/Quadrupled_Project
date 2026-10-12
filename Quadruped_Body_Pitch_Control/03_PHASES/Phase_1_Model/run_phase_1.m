function run_phase_1()
% RUN_PHASE_1 Standalone launcher for Phase 1: Physical Model & Initial Response.
%
% Dynamically ensures project paths are loaded and executes Phase 1.
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
fprintf('  RUNNING: Phase 1 - Physical Model & Initial Response\n');
fprintf('============================================================\n');

% Execute Phase 1
phase_1_model;

fprintf('\n>>> Phase 1 completed successfully.\n\n');
end
