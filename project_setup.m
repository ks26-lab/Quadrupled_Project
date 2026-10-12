function varargout = project_setup(varargin)
% PROJECT_SETUP Root forwarding setup script for Quadruped Body-Pitch Control Project.
%
% Forwards execution to Quadruped_Body_Pitch_Control/01_SETUP/project_setup.m.

root_dir = fileparts(mfilename('fullpath'));
setup_dir = fullfile(root_dir, 'Quadruped_Body_Pitch_Control', '01_SETUP');

if exist(setup_dir, 'dir')
    addpath(setup_dir);
    if nargout > 0
        [varargout{1:nargout}] = project_setup(varargin{:});
    else
        project_setup(varargin{:});
    end
else
    error('project_setup:NotFound', 'Could not locate Quadruped_Body_Pitch_Control/01_SETUP directory.');
end
end
