function run_phase_4_6()
% RUN_PHASE_4_6 Standalone launcher for Phase 4.6 (Phase 6): One-Leg Contact-Loss / Fault Analysis.
%
% Dynamically ensures project paths are loaded, verifies prerequisites,
% and executes Phase 4.6 / Phase 6 contact-loss fault analysis.
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
fprintf('  RUNNING: Phase 4.6 (Phase 6) - One-Leg Contact-Loss Analysis\n');
fprintf('============================================================\n');

% Execute Phase 4.6 implementation
phase_4_6_contact_loss;

fprintf('\n>>> Phase 4.6 (Phase 6) completed successfully.\n\n');
end
