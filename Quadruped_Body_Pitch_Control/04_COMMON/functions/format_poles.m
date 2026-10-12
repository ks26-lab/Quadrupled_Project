function p = format_poles(sys, label)
% FORMAT_POLES Formats and prints pole locations for continuous-time systems.
%
% Usage:
%   p = format_poles(sys)
%   p = format_poles(sys, 'Closed-Loop System')

if nargin < 2
    label = 'System';
end

p = pole(sys);
fprintf('\n--- %s Poles ---\n', label);
for i = 1:numel(p)
    if imag(p(i)) == 0
        fprintf('  Pole %d: %10.4f\n', i, real(p(i)));
    else
        fprintf('  Pole %d: %10.4f %+10.4fi\n', i, real(p(i)), imag(p(i)));
    end
end
fprintf('---------------------\n\n');
end
