function info = step_metrics(sys, sys_name)
% STEP_METRICS Formats and displays standard closed-loop step response metrics.
%
% Usage:
%   info = step_metrics(sys)
%   info = step_metrics(sys, 'PI Controller')

if nargin < 2
    sys_name = 'System';
end

info = stepinfo(sys);

fprintf('\n===== %s PERFORMANCE METRICS =====\n', upper(sys_name));
fprintf('Rise Time     : %.4f s\n', info.RiseTime);
fprintf('Settling Time : %.4f s\n', info.SettlingTime);
fprintf('Overshoot     : %.4f %%\n', info.Overshoot);
fprintf('Peak          : %.4f rad\n', info.Peak);
fprintf('Peak Time     : %.4f s\n', info.PeakTime);
fprintf('===============================================\n\n');
end
