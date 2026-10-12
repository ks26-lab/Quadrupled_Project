function run_phase_4_7()
% RUN_PHASE_4_7 Standalone launcher for Phase 4.7 (Phase 7): Robustness & Model Uncertainty Analysis.
%
% Dynamically ensures project paths are loaded, verifies prerequisites,
% and executes Phase 7 model uncertainty sweeps, pole migration, and final validation.
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
fprintf('  RUNNING: Phase 7 (Phase 4.7) - Robustness & Model Uncertainty\n');
fprintf('============================================================\n');

% Execute Phase 7 implementation
phase_4_7_robustness;

fprintf('\n>>> Phase 7 (Phase 4.7) completed successfully.\n\n');
end
