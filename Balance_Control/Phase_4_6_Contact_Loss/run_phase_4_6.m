function varargout = run_phase_4_6(varargin)
% Phase 4.6 Contact Loss Forwarder
root_dir = fileparts(fileparts(fileparts(mfilename('fullpath'))));
target = fullfile(root_dir, 'Quadruped_Body_Pitch_Control', '03_PHASES', 'Phase_4_6_Contact_Loss', 'run_phase_4_6.m');
if exist(target, 'file')
    run(target);
else
    error('Target phase file not found: %s', target);
end
end
