function print_banner(title_str, subtitle_str)
% PRINT_BANNER Prints a formatted terminal header banner.
%
% Usage:
%   print_banner('Phase 3', 'Feedback Controller Design')

fprintf('\n');
fprintf('============================================================\n');
fprintf('  %s\n', upper(title_str));
if nargin > 1 && ~isempty(subtitle_str)
    fprintf('  %s\n', subtitle_str);
end
fprintf('============================================================\n\n');
end
