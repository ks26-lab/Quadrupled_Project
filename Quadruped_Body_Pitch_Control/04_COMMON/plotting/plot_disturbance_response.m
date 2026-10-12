function h = plot_disturbance_response(t, theta_nom, theta_dist, title_str, legend_dist)
% PLOT_DISTURBANCE_RESPONSE Helper to plot pitch response under external torque disturbance.
%
% Usage:
%   plot_disturbance_response(t, theta_nominal, theta_total, 'Disturbance Response', 'With Disturbance')

if nargin < 4 || isempty(title_str)
    title_str = 'Disturbance Rejection Response';
end
if nargin < 5 || isempty(legend_dist)
    legend_dist = 'Disturbed response';
end

h = figure('Name', title_str);
plot(t, theta_nom, 'LineWidth', 1.5);
hold on;
plot(t, theta_dist, 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('Body Pitch (rad)');
title(title_str);
legend('Nominal response', legend_dist, 'Location', 'best');
hold off;
end
