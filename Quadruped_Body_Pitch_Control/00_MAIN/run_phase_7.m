function run_phase_7()
% RUN_PHASE_7 Universal top-level launcher for Phase 7 (Robustness & Model Uncertainty Analysis).
this_dir = fileparts(mfilename('fullpath'));
setup_dir = fullfile(fileparts(this_dir), '01_SETUP');
if exist(fullfile(setup_dir, 'project_setup.m'), 'file')
    addpath(setup_dir);
    project_setup(true);
end
run_phase_4_7();
end
