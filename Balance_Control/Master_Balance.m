function varargout = Master_Balance(varargin)
% MASTER_BALANCE Entry launcher for Balance Control Subsystem.
%
% Forwards execution to Quadruped_Body_Pitch_Control launcher while
% preserving all validated pitch control models, controllers, and tests.

root_dir = fileparts(fileparts(mfilename('fullpath')));
setup_dir = fullfile(root_dir, 'Quadruped_Body_Pitch_Control', '01_SETUP');
main_dir  = fullfile(root_dir, 'Quadruped_Body_Pitch_Control', '00_MAIN');

if exist(setup_dir, 'dir')
    addpath(setup_dir);
    project_setup(false);
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
