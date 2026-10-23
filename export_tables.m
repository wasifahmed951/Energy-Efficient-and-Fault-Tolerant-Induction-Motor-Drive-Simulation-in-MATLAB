%% export_tables.m  -  writes the remaining MATLAB results as readable CSV tables
% Run from the "Electrical Drive" folder:   export_tables
% Writes (in results\):
%   table_rq1_rq2.csv      section 5: ZN / GA / PSO x IFOC / DTC x test cycles (all metrics)
%   table_tuning_runs.csv  section 4: every GA / PSO run (drive, method, seed, J, gains)
%   table_tuning_summary.csv          mean / std / min of J per drive and optimiser
% and prints them together with the DTC band calibration (section 2) and the fault study (section 7).

function export_tables()
setup_project();
root = fileparts(mfilename('fullpath'));
rd = fullfile(root, 'results');

% ---- section 5: RQ1 + RQ2 comparison ----
S = load(fullfile(rd, 'comparison_rq1_rq2.mat'), 'rows', 'Mm', 'names');
vn = matlab.lang.makeUniqueStrings(matlab.lang.makeValidName(cellstr(S.names)));
T5 = [cell2table(S.rows, 'VariableNames', {'drive', 'tuning', 'cycle'}), array2table(S.Mm, 'VariableNames', vn)];
writetable(T5, fullfile(rd, 'table_rq1_rq2.csv'));
fprintf('\n========== SECTION 5: ZN / GA / PSO comparison (RQ1, RQ2) ==========\n');
disp(T5);

% ---- section 4: every tuning run ----
U = load(fullfile(rd, 'tuning_runs.mat'), 'tun');
n = numel(U.tun);  C = cell(n, 6);
for k = 1:n
    o = U.tun(k).out;
    nev = NaN;
    for f = {'n_eval', 'nevals', 'evals', 'neval'}
        if isfield(o, f{1}), nev = double(o.(f{1})); break; end
    end
    C(k, :) = {U.tun(k).drive, upper(U.tun(k).method), double(o.seed), double(o.J), nev, mat2str(double(o.x(:)'), 5)};
end
T4 = cell2table(C, 'VariableNames', {'drive', 'method', 'seed', 'J', 'n_eval', 'gains'});
Jcols = 'J';
f_rs = fullfile(rd, 'tuning_rescored.mat');
if exist(f_rs, 'file')
    % J re-evaluated with the corrected rms-ripple integration (tuning/rescore_tuning.m, 17 Oct 2026)
    R = load(f_rs, 'T');
    T4.J_corrected = R.T.J_new(1:n);
    Jcols = {'J', 'J_corrected'};
end
writetable(T4, fullfile(rd, 'table_tuning_runs.csv'));
G = groupsummary(T4, {'drive', 'method'}, {'mean', 'std', 'min'}, Jcols);
writetable(G, fullfile(rd, 'table_tuning_summary.csv'));
fprintf('\n========== SECTION 4: GA / PSO runs ==========\n');
disp(T4);  disp(G);
fprintf('fields stored per tuning run: %s\n', strjoin(fieldnames(U.tun(1).out)', ', '));

% ---- section 3: ZN gains ----
Z = load(fullfile(rd, 'zn_params.mat'));
fprintf('\n========== SECTION 3: Ziegler-Nichols ==========\n');
fprintf('IFOC ZN: Kp %.4g  Ki %.4g\n', Z.Pi_zn.ctrl.Kp_w, Z.Pi_zn.ctrl.Ki_w);
fprintf('DTC  ZN: Kp %.4g  Ki %.4g\n', Z.Pd_zn.ctrl.Kp_w, Z.Pd_zn.ctrl.Ki_w);
disp(Z.zn_ifoc);  disp(Z.zn_dtc);

% ---- section 2: DTC band calibration ----
B = load(fullfile(rd, 'dtc_bands.mat'), 'band_info');
fprintf('\n========== SECTION 2: DTC band calibration ==========\n');
disp(B.band_info);

% ---- section 7: fault study ----
F = readtable(fullfile(rd, 'fault_study.csv'));
fprintf('\n========== SECTION 7: fault study (RQ4) ==========\n');
disp(F);

fprintf('\nDone. Tables saved in results\\: table_rq1_rq2.csv, table_tuning_runs.csv, table_tuning_summary.csv,\n');
fprintf('fault_study.csv (the printed text above is also in the log).\n');
end
