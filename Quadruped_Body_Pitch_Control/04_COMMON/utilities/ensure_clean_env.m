function ensure_clean_env(close_figs)
% ENSURE_CLEAN_ENV Prepares MATLAB environment safely.
%
% Closes open figures if requested and formats the command window.
% Preserves critical workspace variables to avoid workspace fragility.

if nargin < 1
    close_figs = false;
end

if close_figs
    close all;
end

format compact;
format shortG;
end
