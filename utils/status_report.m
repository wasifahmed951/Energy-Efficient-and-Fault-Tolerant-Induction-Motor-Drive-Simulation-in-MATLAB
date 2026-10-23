function status_report(res_dir, ds_dir)
%STATUS_REPORT  Show which main_run_all sections are finished, from the
%   result files on disk. Run it any time:  status_report
if nargin < 1
    root = fileparts(fileparts(mfilename('fullpath')));
    res_dir = fullfile(root, 'results');  ds_dir = fullfile(root, 'dataset');
end
S = {
  '2  DTC band calibration',       fullfile(res_dir, 'dtc_bands.mat')
  '3  Ziegler-Nichols',            fullfile(res_dir, 'zn_params.mat')
  '4  GA/PSO tuning',              fullfile(res_dir, 'tuning_runs.mat')
  '5  ZN/GA/PSO comparison',       fullfile(res_dir, 'comparison_rq1_rq2.mat')
  '6a flux sweep IFOC',            fullfile(res_dir, 'flux_sweep_IFOC.csv')
  '6b flux sweep DTC',             fullfile(res_dir, 'flux_sweep_DTC.csv')
  '6c efficiency compare IFOC',    fullfile(res_dir, 'efficiency_IFOC.csv')
  '6d efficiency compare DTC',     fullfile(res_dir, 'efficiency_DTC.csv')
  '7  fault study',                fullfile(res_dir, 'fault_study.csv')
  '8  fault dataset',              fullfile(ds_dir, 'fault_windows.mat')
  '9  AGV missions',               fullfile(res_dir, 'agv_results.mat')
  };
fprintf('\n---- main_run_all status ----\n');
for k = 1:size(S,1)
    if exist(S{k,2}, 'file')
        d = dir(S{k,2});
        fprintf('  DONE     %-28s %s\n', S{k,1}, d.date);
    else
        fprintf('  not yet  %s\n', S{k,1});
    end
end
ck = fullfile(res_dir, 'tuning_partial.mat');
if ~exist(fullfile(res_dir, 'tuning_runs.mat'), 'file') && exist(ck, 'file')
    T = load(ck);  fprintf('  (GA/PSO checkpoint: %d of 20 runs done)\n', numel(T.tun));
end
fprintf('-----------------------------\n\n');
end
