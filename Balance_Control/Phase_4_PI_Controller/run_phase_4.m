function varargout = run_phase_4(varargin)
% Phase 4 PI Controller Forwarder
root_dir = fileparts(fileparts(fileparts(mfilename('fullpath'))));
target = fullfile(root_dir, 'Quadruped_Body_Pitch_Control', '03_PHASES', 'Phase_4_PI_Controller', 'run_phase_4.m');
if exist(target, 'file')
    run(target);
else
    error('Target phase file not found: %s', target);
end
end
