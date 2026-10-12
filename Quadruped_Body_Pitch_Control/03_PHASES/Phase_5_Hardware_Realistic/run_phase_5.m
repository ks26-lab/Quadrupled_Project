function run_phase_5()
% RUN_PHASE_5 Standalone launcher for Phase 5: Hardware-Realistic SISO Simulation.
%
% Dynamically ensures project paths are loaded, verifies prerequisites,
% and executes Phase 5 hardware non-idealities progression.
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
fprintf('  RUNNING: Phase 5 - Hardware-Realistic SISO Simulation\n');
fprintf('============================================================\n');

% Execute Phase 5 implementation
phase_5_hardware_realistic;

fprintf('\n>>> Phase 5 completed successfully.\n\n');
end
