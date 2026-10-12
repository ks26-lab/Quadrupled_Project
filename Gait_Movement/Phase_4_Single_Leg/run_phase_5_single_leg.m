function run_phase_5_single_leg()
% Phase 5 Single Leg Foundation Runner
root_dir = fileparts(fileparts(mfilename('fullpath')));
run(fullfile(root_dir, 'Master_Gait.m'));
end
