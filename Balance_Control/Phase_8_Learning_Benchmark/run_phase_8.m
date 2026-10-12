function varargout = run_phase_8(varargin)
% RUN_PHASE_8 Root forwarder for Phase 8 Learning Control Benchmark.
%
% Forwards execution to Quadruped_Body_Pitch_Control/03_PHASES/Phase_8_Learning_Control_Benchmark/run_phase_8.m

root_dir = fileparts(fileparts(fileparts(mfilename('fullpath'))));
target = fullfile(root_dir, 'Quadruped_Body_Pitch_Control', '03_PHASES', 'Phase_8_Learning_Control_Benchmark', 'run_phase_8.m');

if exist(target, 'file')
    if nargout > 0
        [varargout{1:nargout}] = run(target);
    else
        run(target);
    end
else
    error('run_phase_8:NotFound', 'Target phase file not found: %s', target);
end
end
