function varargout = run_phase_2(varargin)
% Phase 2 OpenLoop Forwarder
root_dir = fileparts(fileparts(fileparts(mfilename('fullpath'))));
target = fullfile(root_dir, 'Quadruped_Body_Pitch_Control', '03_PHASES', 'Phase_2_SISO', 'run_phase_2.m');
if exist(target, 'file')
    run(target);
else
    error('Target phase file not found: %s', target);
end
end
