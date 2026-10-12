function run_phase_2()
% RUN_PHASE_2 Standalone launcher for Phase 2: SISO Plant & Open-Loop Analysis.
%
% Dynamically ensures project paths are loaded, initializes required plant
% dependency G if missing, and executes Phase 2.
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
fprintf('  RUNNING: Phase 2 - SISO Plant & Open-Loop Analysis\n');
fprintf('============================================================\n');

% Execute Phase 2
phase_2_siso;

fprintf('\n>>> Phase 2 completed successfully.\n\n');
end
