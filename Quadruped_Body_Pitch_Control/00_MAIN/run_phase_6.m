function run_phase_6()
% RUN_PHASE_6 Universal top-level launcher for Phase 6 (Phase 4.6 Contact Loss Analysis).
this_dir = fileparts(mfilename('fullpath'));
setup_dir = fullfile(fileparts(this_dir), '01_SETUP');
if exist(fullfile(setup_dir, 'project_setup.m'), 'file')
    addpath(setup_dir);
    project_setup(true);
end
run_phase_4_6();
end
