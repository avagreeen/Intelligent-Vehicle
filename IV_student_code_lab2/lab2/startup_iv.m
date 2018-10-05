% --------------------------------------------------------
% Intelligent Vehicles Lab Assignment
% --------------------------------------------------------
% Julian Kooij, Delft University of Technology
%
% This adds the relevant utility directories to the matlab path.
% Run it to add all the required subdirectories as (absolute!) paths 
% to Matlab's search path.

IV_BASE_PATH = fileparts(fileparts(mfilename('fullpath')));

addpath(IV_BASE_PATH); % this directory

addpath(fullfile(IV_BASE_PATH, 'utils')); % generic useful functions
addpath(fullfile(IV_BASE_PATH, 'utils_iv')); % functions for IV course

% disable confusing warnings on 'axis equal' command in Matlab R2016a.
warning('off', 'MATLAB:nargchk:deprecated');
