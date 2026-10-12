function h = plot_step_response(sys_list, labels, title_str, t_end)
% PLOT_STEP_RESPONSE Helper to plot comparative step responses.
%
% Usage:
%   plot_step_response({T_fast, T_final}, {'Fastest', 'Final'}, 'Candidate Comparison')

if nargin < 3 || isempty(title_str)
    title_str = 'Step Response Comparison';
end

h = figure('Name', title_str);
hold on;
for i = 1:numel(sys_list)
    if nargin >= 4 && ~isempty(t_end)
        t = 0:0.01:t_end;
        step(sys_list{i}, t);
    else
        step(sys_list{i});
    end
end
grid on;
title(title_str);
xlabel('Time (s)');
ylabel('Body Pitch \theta (rad)');

if nargin >= 2 && ~isempty(labels)
    legend(labels, 'Location', 'best');
end
hold off;
end
