%% rerun_after_fix.m  -  re-runs ONLY the results affected by the 17 Oct 2026 fixes
%
%  Fix 1 (sim/unpack_sim_output.m): Simulink log rows are now stamped k*Ts like the
%         plain-MATLAB engine. Affects: detection times (+0.1 ms), ITAE (slightly),
%         and the pre-fault torque ripple of the open-phase rows in fault_study.csv.
%  Fix 2 (core/plant_step.m): Te^2, ia^2 and the copper losses are integrated exactly
%         (Simpson) instead of with a trapezoid that over-estimated them. Affects:
%         rms torque ripple, total THD, copper-loss breakdown, and the tuning cost J.
%  NOT affected (not re-run): DTC bands, ZN, GA/PSO search (their J is re-scored
%  instead), flux sweep, fault dataset, Python ML, AGV missions, efficiency savings.
%
%  The old result files are MOVED to results\before_fix_17oct\ (nothing is deleted).
%  Run from the "Electrical Drive" folder:   rerun_after_fix      (about 1-2 hours)

bdclose all;  clear functions;
setup_project();
rf_root = fileparts(mfilename('fullpath'));
rf_rd = fullfile(rf_root, 'results');
rf_bk = fullfile(rf_rd, 'before_fix_17oct');
if ~exist(rf_bk, 'dir'), mkdir(rf_bk); end
rf_files = {'comparison_rq1_rq2.mat', 'table_rq1_rq2.csv', 'efficiency_IFOC.csv', 'efficiency_DTC.csv', ...
    'fault_study.csv', 'fairness_checks.mat', 'table_fsw_matched.csv', ...
    'efficiency_ml_IFOC.csv', 'efficiency_ml_DTC.csv', 'dynamic_ml.csv', 'dynamic_ml.mat', ...
    'table_tuning_runs.csv', 'table_tuning_summary.csv'};
for rf_k = 1:numel(rf_files)
    rf_f = fullfile(rf_rd, rf_files{rf_k});
    if exist(rf_f, 'file'), movefile(rf_f, fullfile(rf_bk, rf_files{rf_k})); end
end
fprintf('Old result files moved to %s\n', rf_bk);
% Simulink caches of the MATLAB Function blocks must see the new plant_step.m
rf_slprj = fullfile(rf_root, 'slprj');
if exist(rf_slprj, 'dir'), rmdir(rf_slprj, 's'); end
delete(fullfile(rf_root, '*.slxc'));
% sections 5 (RQ1/RQ2 table), 6 (efficiency tables) and 7 (fault study); the rest is skipped
main_run_all;
% fairness checks (CHECK 1 re-runs PSO with wider bounds, CHECK 2 the matched-frequency table)
fairness_checks;
% section 10: A (efficiency with ML flux) and B (load step); C (AGV) is skipped
run_ml_flux_all;
% tuning cost J re-scored with the corrected ripple, and the optimum check
rescore_tuning;
% refresh the readable CSV tables
export_tables;
fprintf('\nRERUN AFTER FIX FINISHED. Send the printed tables (or the log files in results\\).\n');
