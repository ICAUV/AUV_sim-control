% startup_auv1.m
% Adds matlab/ and matlab/mss/ to the MATLAB path. Paths are resolved relative
% to this file, so it works wherever the repo is checked out.
% matlab/tests/ is deliberately NOT added, which keeps the reference remus100
% off the normal path.

matlabRoot = fileparts(mfilename('fullpath'));
addpath(matlabRoot);
addpath(fullfile(matlabRoot, 'mss'));
clear matlabRoot
