function varargout = run_phase_1(varargin)
% Phase 1 Model Forwarder
root_dir = fileparts(fileparts(fileparts(mfilename('fullpath'))));
target = fullfile(root_dir, 'Quadruped_Body_Pitch_Control', '03_PHASES', 'Phase_1_Model', 'run_phase_1.m');
if exist(target, 'file')
    run(target);
else
    error('Target phase file not found: %s', target);
end
end
