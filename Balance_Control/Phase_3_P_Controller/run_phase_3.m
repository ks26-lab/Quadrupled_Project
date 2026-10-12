function varargout = run_phase_3(varargin)
% Phase 3 P-Controller Forwarder
root_dir = fileparts(fileparts(fileparts(mfilename('fullpath'))));
target = fullfile(root_dir, 'Quadruped_Body_Pitch_Control', '03_PHASES', 'Phase_3_P_Controller', 'run_phase_3.m');
if exist(target, 'file')
    run(target);
else
    error('Target phase file not found: %s', target);
end
end
