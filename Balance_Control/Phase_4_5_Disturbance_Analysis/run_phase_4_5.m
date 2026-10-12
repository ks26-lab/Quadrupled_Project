function varargout = run_phase_4_5(varargin)
% Phase 4.5 Disturbance Rejection Forwarder
root_dir = fileparts(fileparts(fileparts(mfilename('fullpath'))));
target = fullfile(root_dir, 'Quadruped_Body_Pitch_Control', '03_PHASES', 'Phase_4_5_Disturbance_Frequency', 'run_phase_4_5.m');
if exist(target, 'file')
    run(target);
else
    error('Target phase file not found: %s', target);
end
end
