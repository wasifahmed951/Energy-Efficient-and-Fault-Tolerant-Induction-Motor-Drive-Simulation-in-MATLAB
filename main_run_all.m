%MAIN_RUN_ALL  Runs every thesis experiment in order.
%
%   RESUMABLE: every section saves its result in results/ or dataset/, and a
%   section whose result file already exists is SKIPPED (its result is loaded
%   instead). If MATLAB crashes, just start main_run_all again - it continues
%   from where it stopped. Long batches (GA/PSO, flux sweep, fault dataset)
%   also save checkpoints inside the section.
%   To redo a section, delete its result file (listed in each section).
%
%   ENGINE: 'simulink' runs the Simulink models (IM_DRIVE.slx, AGV_MISSION.slx);
%           'matlab' runs the same code in a plain loop (no Simulink needed).
%   PARALLEL: true uses parsim / parfor when Parallel Computing Toolbox exists.

clear; clc;
setup_project();
ENGINE   = 'simulink';
PARALLEL = true;
root = fileparts(mfilename('fullpath'));
res_dir = fullfile(root, 'results');
ds_dir  = fullfile(root, 'dataset');
diary(fullfile(res_dir, sprintf('log_%s.txt', datestr(now, 'yyyymmdd_HHMM'))));   % keeps the Command Window text
f_done = @(f) exist(fullfile(res_dir, f), 'file') == 2;
status_report(res_dir, ds_dir);

%% 0. Build the Simulink models (once, or after changing the MATLAB Function blocks)
if ~exist(fullfile(root, 'simulink', 'IM_DRIVE.slx'), 'file'), build_drive_model('IM_DRIVE'); end
if ~exist(fullfile(root, 'simulink', 'AGV_MISSION.slx'), 'file'), build_agv_model('AGV_MISSION'); end

%% 1. Quick check: both drives on the tuning cycle with default gains (plots only)
Pi = default_params('IFOC');
Pd = default_params('DTC');
if ~f_done('dtc_bands.mat')
    ri = run_sim(Pi, make_scenario('tuning', Pi), ENGINE);
    rd = run_sim(Pd, make_scenario('tuning', Pd), ENGINE);
    plot_run(ri, 'IFOC, default gains');
    plot_run(rd, 'DTC, default gains');
end

%% 2. Fairness: calibrate DTC bands to the IFOC switching frequency (5 kHz)
%   result: results/dtc_bands.mat
if f_done('dtc_bands.mat')
    load(fullfile(res_dir, 'dtc_bands.mat'), 'band_info');
    Pd.ctrl.h_T = band_info.h_T;  Pd.ctrl.h_psi = band_info.h_psi;
    fprintf('[2] skipped (dtc_bands.mat exists)\n');
else
    [Pd, band_info] = calibrate_dtc_bands(Pd, Pi.inv.fsw, ENGINE);
    save(fullfile(res_dir, 'dtc_bands.mat'), 'band_info');
end

%% 3. Ziegler-Nichols baseline (relay test)
%   result: results/zn_params.mat
if f_done('zn_params.mat')
    load(fullfile(res_dir, 'zn_params.mat'), 'Pi_zn', 'Pd_zn', 'zn_ifoc', 'zn_dtc');
    fprintf('[3] skipped (zn_params.mat exists)\n');
else
    [Pi_zn, zn_ifoc] = run_zn_relay(Pi, ENGINE);
    [Pd_zn, zn_dtc]  = run_zn_relay(Pd, ENGINE);
    save(fullfile(res_dir, 'zn_params.mat'), 'Pi_zn', 'Pd_zn', 'zn_ifoc', 'zn_dtc');
end

%% 4. GA and PSO tuning: equal budget (600 evaluations), 5 seeds each
%   results: results/tuning_runs.mat, results/tuned_params.mat
%   checkpoint: results/tuning_partial.mat (one entry per finished run)
SEEDS = 1:5;  BUDGET = 600;  POP = 20;
if f_done('tuning_runs.mat')
    load(fullfile(res_dir, 'tuning_runs.mat'), 'tun');
    fprintf('[4] skipped (tuning_runs.mat exists)\n');
else
    ck = fullfile(res_dir, 'tuning_partial.mat');
    tun = struct('drive', {}, 'method', {}, 'out', {});
    if exist(ck, 'file'), load(ck, 'tun'); fprintf('[4] resuming: %d of 20 runs already done\n', numel(tun)); end
    for drv = {'IFOC', 'DTC'}
        if strcmp(drv{1}, 'IFOC'), P0 = Pi_zn; else, P0 = Pd_zn; end
        for meth = {'ga', 'pso'}
            for s = SEEDS
                if any(arrayfun(@(t) strcmp(t.drive, drv{1}) && strcmp(t.method, meth{1}) ...
                        && t.out.seed == s, tun))
                    continue;                                  % already done before the crash
                end
                fprintf('\n=== %s %s seed %d ===\n', drv{1}, upper(meth{1}), s);
                o = run_tuning(P0, meth{1}, struct('budget', BUDGET, 'pop', POP, 'seed', s, ...
                    'engine', ENGINE, 'parallel', PARALLEL));
                tun(end+1) = struct('drive', drv{1}, 'method', meth{1}, 'out', o); %#ok<SAGROW>
                save(ck, 'tun');
            end
        end
    end
    save(fullfile(res_dir, 'tuning_runs.mat'), 'tun', 'zn_ifoc', 'zn_dtc');
end
% best gains per drive (across both optimisers) are carried forward
Pi_best = pick_best(tun, 'IFOC', Pi_zn);
Pd_best = pick_best(tun, 'DTC', Pd_zn);
save(fullfile(res_dir, 'tuned_params.mat'), 'Pi_best', 'Pd_best', 'Pi_zn', 'Pd_zn');

%% 5. RQ1 + RQ2: compare ZN / GA / PSO gains for both drives on test cycles a-c
%   result: results/comparison_rq1_rq2.mat
if f_done('comparison_rq1_rq2.mat')
    fprintf('[5] skipped (comparison_rq1_rq2.mat exists)\n');
else
    cycles = {'step_rated', 'load_step', 'reversal'};
    rows = {};  M_all = {};
    for drv = {'IFOC', 'DTC'}
        for tuneby = {'ZN', 'GA', 'PSO'}
            P = select_params(drv{1}, tuneby{1}, Pi_zn, Pd_zn, tun);
            for c = cycles
                r = run_sim(P, make_scenario(c{1}, P), ENGINE);
                M_all{end+1} = compute_metrics(r); %#ok<SAGROW>
                rows(end+1, :) = {drv{1}, tuneby{1}, c{1}}; %#ok<SAGROW>
                clear r
            end
        end
    end
    [Mm, names] = metrics_table(M_all);
    save(fullfile(res_dir, 'comparison_rq1_rq2.mat'), 'rows', 'Mm', 'names');
    fprintf('Saved comparison table: %d rows x %d metrics\n', size(Mm,1), size(Mm,2));
end

%% 6. RQ3: flux sweep (data for Python), analytic look-up table, efficiency comparison
%   results: results/flux_sweep_IFOC.csv, flux_sweep_DTC.csv, efficiency_IFOC.csv, efficiency_DTC.csv
if ~f_done('flux_sweep_IFOC.csv'), run_flux_sweep(Pi_best, struct('engine', ENGINE, 'parallel', PARALLEL)); end
if ~f_done('flux_sweep_DTC.csv'),  run_flux_sweep(Pd_best, struct('engine', ENGINE, 'parallel', PARALLEL)); end
Pi_best = build_flux_lut(Pi_best, 'analytic');
Pd_best = build_flux_lut(Pd_best, 'analytic');
% (after Python: Pi_best = build_flux_lut(Pi_best, 'csv', 'results/ml_lut_IFOC.csv'), or load MLP weights)
if ~f_done('efficiency_IFOC.csv'), run_efficiency_compare(Pi_best, struct('engine', ENGINE, 'parallel', PARALLEL)); end
if ~f_done('efficiency_DTC.csv'),  run_efficiency_compare(Pd_best, struct('engine', ENGINE, 'parallel', PARALLEL)); end
save(fullfile(res_dir, 'tuned_params.mat'), 'Pi_best', 'Pd_best', 'Pi_zn', 'Pd_zn');

%% 7. RQ4: fault study
%   result: results/fault_study.csv
if ~f_done('fault_study.csv')
    run_fault_study(Pi_best, Pd_best, struct('engine', ENGINE, 'parallel', PARALLEL));
else
    fprintf('[7] skipped (fault_study.csv exists)\n');
end

%% 8. Fault dataset for the Python classifiers (Mamba2, XGBoost, KAN, 1-D CNN)
%   result: dataset/fault_windows.mat
if ~exist(fullfile(ds_dir, 'fault_windows.mat'), 'file')
    export_fault_dataset(Pi_best, Pd_best, struct('engine', ENGINE, 'parallel', PARALLEL));
else
    fprintf('[8] skipped (fault_windows.mat exists)\n');
end

%% 9. RQ6: AGV missions (both drives, rated vs optimised flux, faults)
%   result: results/agv_results.mat   checkpoint: results/agv_partial.mat
if f_done('agv_results.mat')
    fprintf('[9] skipped (agv_results.mat exists)\n');
else
    ck = fullfile(res_dir, 'agv_partial.mat');
    agv = cell(0, 4);
    if exist(ck, 'file'), load(ck, 'agv'); end
    for drv = {'IFOC', 'DTC'}
        if strcmp(drv{1}, 'IFOC'), P0 = Pi_best; else, P0 = Pd_best; end
        for fm = [1 3]
            for fault = [0 1 2 3]
                if any(strcmp(agv(:,1), drv{1}) & cell2mat(agv(:,2)) == fm & cell2mat(agv(:,3)) == fault)
                    continue;
                end
                P = P0;  P.ctrl.flux_mode = fm;  P.agv.fault_type = fault;
                fprintf('%s  starting %s flux %d fault %d ...\n', datestr(now, 'HH:MM:SS'), drv{1}, fm, fault);
                diary off; diary on;                     % flush the log, so a hang shows where
                try
                    r = run_agv(P, ENGINE);
                catch err
                    % one failed mission must not stop the others: record it as not completed
                    fprintf(2, '%s flux %d fault %d FAILED: %s\n', drv{1}, fm, fault, err.message);
                    r.M = struct('completed', 0, 'E_Wh', NaN, 'e_cross_max_cm', NaN, 'error', err.message);
                end
                agv(end+1, :) = {drv{1}, fm, fault, r.M}; %#ok<SAGROW>
                save(ck, 'agv');
                fprintf('%s flux %d fault %d: done %d, %.2f Wh, e_max %.1f cm\n', drv{1}, fm, fault, ...
                    r.M.completed, r.M.E_Wh, r.M.e_cross_max_cm);
                clear r
            end
        end
    end
    save(fullfile(res_dir, 'agv_results.mat'), 'agv');
end
fprintf('\nALL SECTIONS FINISHED\n');
status_report(res_dir, ds_dir);
diary off

%% ---------------- local helpers ----------------
function P = pick_best(tun, drv, P0)
best = Inf;  P = P0;
for k = 1:numel(tun)
    if strcmp(tun(k).drive, drv) && tun(k).out.J < best
        best = tun(k).out.J;  P = apply_gains(P0, tun(k).out.x);
    end
end
end

function P = select_params(drv, tuneby, Pi_zn, Pd_zn, tun)
if strcmp(drv, 'IFOC'), P = Pi_zn; else, P = Pd_zn; end
if strcmp(tuneby, 'ZN'), return; end
best = Inf;
for k = 1:numel(tun)
    if strcmp(tun(k).drive, drv) && strcmpi(tun(k).method, tuneby) && tun(k).out.J < best
        best = tun(k).out.J;  x = tun(k).out.x;
    end
end
P = apply_gains(P, x);
end
