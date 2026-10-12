function varargout = run_project(varargin)
% RUN_PROJECT Root forwarding launcher for Quadruped Body-Pitch Control Project.
%
% Forwards execution to Quadruped_Body_Pitch_Control/00_MAIN/run_project.m.

root_dir = fileparts(mfilename('fullpath'));
main_dir = fullfile(root_dir, 'Quadruped_Body_Pitch_Control', '00_MAIN');
setup_dir = fullfile(root_dir, 'Quadruped_Body_Pitch_Control', '01_SETUP');

if exist(setup_dir, 'dir')
    addpath(setup_dir);
end
if exist(main_dir, 'dir')
    addpath(main_dir);
end

if nargout > 0
    [varargout{1:nargout}] = run_project(varargin{:});
else
    run_project(varargin{:});
end
end
