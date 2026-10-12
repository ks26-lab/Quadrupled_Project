function varargout = run_phase_4_7(varargin)
% Phase 4.7 Robustness Forwarder
root_dir = fileparts(fileparts(fileparts(mfilename('fullpath'))));
target = fullfile(root_dir, 'Quadruped_Body_Pitch_Control', '03_PHASES', 'Phase_4_7_Robustness', 'run_phase_4_7.m');
if exist(target, 'file')
    run(target);
else
    error('Target phase file not found: %s', target);
end
end
