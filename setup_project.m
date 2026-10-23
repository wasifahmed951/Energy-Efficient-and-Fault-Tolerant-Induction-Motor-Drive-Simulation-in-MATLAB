function setup_project()
%SETUP_PROJECT  Add all project folders to the MATLAB path. Run this first
%   (main_run_all calls it, and parsim workers call it too).
root = fileparts(mfilename('fullpath'));
folders = {'params','core','sim','analysis','tuning','efficiency','faults','agv','simulink','utils','blocks'};
for i = 1:numel(folders)
    addpath(fullfile(root, folders{i}));
end
if ~exist(fullfile(root, 'results'), 'dir'), mkdir(fullfile(root, 'results')); end
if ~exist(fullfile(root, 'dataset'), 'dir'), mkdir(fullfile(root, 'dataset')); end
end
