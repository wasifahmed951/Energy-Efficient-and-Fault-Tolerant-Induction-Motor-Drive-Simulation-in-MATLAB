%% run_ml_flux_all.m  -  SECTION 10: ML flux reference in both drives (RQ3, RQ4, RQ6)
%
% Uses the TUNED drives from main_run_all (results/tuned_params.mat: Pi_best, Pd_best,
% which already contain the analytic look-up table from section 6) and the ML models
% chosen in python/01_flux_models.ipynb (lowest CV error among good-fit models):
%     IFOC -> KAN  (python_out/lut_kan_IFOC.csv, used as look-up table)
%     DTC  -> MLP  (python_out/mlp_DTC.mat, flux_mode 4)
%
% Three flux strategies are compared in every test:
%     1 = rated flux | 2 = analytic loss-model LUT (section 6) | 3 = ML model
%
%   A. Steady-state efficiency   (RQ3)  -> results/efficiency_ml_IFOC.csv, efficiency_ml_DTC.csv
%   B. Dynamic cost: load step   (RQ3)  -> results/dynamic_ml.csv
%   C. AGV missions with ML flux, healthy + 3 faults (RQ4, RQ6)
%                                       -> results/agv_ml_results.mat, results/agv_all_flux.csv
%
% RESUMABLE like main_run_all: a part whose result file exists is skipped.
% Run from the "Electrical Drive" folder:   run_ml_flux_all

function run_ml_flux_all()
clc;
setup_project();
ENGINE   = 'simulink';
PARALLEL = true;
root    = fileparts(mfilename('fullpath'));
res_dir = fullfile(root, 'results');
py_dir  = fullfile(root, 'python_out');
diary(fullfile(res_dir, sprintf('log_ml_%s.txt', datestr(now, 'yyyymmdd_HHMM'))));
f_done = @(f) exist(fullfile(res_dir, f), 'file') == 2;

%% ---------------- load tuned drives and the ML models ----------------
S = load(fullfile(res_dir, 'tuned_params.mat'), 'Pi_best', 'Pd_best');
Pi = S.Pi_best;  Pd = S.Pd_best;                       % tuned gains + analytic LUT
fprintf('Tuned gains  IFOC: Kp %.3g Ki %.4g | DTC: Kp %.3g Ki %.4g\n', ...
    Pi.ctrl.Kp_w, Pi.ctrl.Ki_w, Pd.ctrl.Kp_w, Pd.ctrl.Ki_w);

% IFOC: KAN exported as a table on the P.lut grid -> replaces the LUT, flux_mode 3
Pi_ml = build_flux_lut(Pi, 'csv', fullfile(py_dir, 'lut_kan_IFOC.csv'));
Pi_ml.ctrl.flux_mode = 3;
% DTC: MLP weights -> P.mlp, flux_mode 4 (copied field by field so the struct keeps its layout)
Mw = load(fullfile(py_dir, 'mlp_DTC.mat'));
Pd_ml = Pd;
for f = fieldnames(Pd.mlp)'
    v = double(Mw.mlp.(f{1}));
    assert(isequal(size(v), size(Pd.mlp.(f{1}))), 'mlp.%s has size %s, expected %s', ...
        f{1}, mat2str(size(v)), mat2str(size(Pd.mlp.(f{1}))));
    Pd_ml.mlp.(f{1}) = v;
end
Pd_ml.ctrl.flux_mode = 4;

D = struct('tag', {'IFOC', 'DTC'}, 'P', {Pi, Pd}, 'Pml', {Pi_ml, Pd_ml}, 'ml_name', {'KAN', 'MLP'});
strat_names = {'rated', 'analytic LUT', 'ML'};


%% ---------------- A. steady-state efficiency (RQ3) ----------------
% speeds 30/60/90 % x loads 10/25/50 % rated (10 % = lowest torque in the training sweep)
wv = [0.3 0.6 0.9];  Tv = [0.10 0.25 0.50];
cols = {'w', 'T', 'strategy', 'psi_ref', 'P_in', 'eta_drive', 'eta_motor', 'P_fe', 'P_cus', 'P_cur', ...
        'Te_ripple_rms', 'thd50', 'saving_pct'};
I = log_index();
for d = 1:2
    fout = ['efficiency_ml_' D(d).tag '.csv'];
    if f_done(fout), fprintf('[A] skipped (%s exists)\n', fout); continue; end
    wr = D(d).P.m.wm_rated;  Tr = D(d).P.m.T_rated;
    Pl = {};  Sl = {};  key = [];
    for a = wv
        for b = Tv
            for s = 1:3
                Ps = pick_strategy(D(d), s);
                Pl{end+1} = Ps; %#ok<SAGROW>
                Sl{end+1} = make_scenario('steady', Ps, struct('w', a*wr, 'T', b*Tr, 'Tend', 1.2)); %#ok<SAGROW>
                key(end+1, :) = [a*wr b*Tr s]; %#ok<SAGROW>
            end
        end
    end
    fprintf('\n[A] %s efficiency: %d runs\n', D(d).tag, numel(Pl));
    red = @(r, i) eff_row(r, i, key, I);
    R = run_batch(Pl, Sl, ENGINE, PARALLEL, struct('reduce', red));
    E = NaN(numel(R), numel(cols));
    for i = 1:numel(R)
        E(i, 1:3) = key(i, :);
        if ~isempty(R{i}), E(i, 1:12) = R{i}; end
    end
    for i = 1:size(E, 1)
        r0 = E(:,1) == E(i,1) & E(:,2) == E(i,2) & E(:,3) == 1;
        E(i, 13) = (E(r0, 5) - E(i, 5))/E(r0, 5)*100;
    end
    write_csv(fullfile(res_dir, fout), cols, E);
    fprintf('Wrote %s\n', fout);
end

%% ---------------- B. dynamic cost: load step 25 % -> 100 % at rated speed (RQ3) ----------------
if f_done('dynamic_ml.csv')
    fprintf('[B] skipped (dynamic_ml.csv exists)\n');
else
    Mall = {};  lab = {};
    for d = 1:2
        for s = 1:3
            Ps = pick_strategy(D(d), s);
            fprintf('[B] %s %s load step ...\n', D(d).tag, strat_names{s});
            r = run_sim(Ps, make_scenario('load_step', Ps), ENGINE);
            Mall{end+1} = compute_metrics(r); %#ok<SAGROW>
            lab(end+1, :) = {D(d).tag, strat_names{s}}; %#ok<SAGROW>
            clear r
        end
    end
    [Mm, names] = metrics_table(Mall);
    Tdyn = [cell2table(lab, 'VariableNames', {'drive', 'flux'}), ...
            array2table(Mm, 'VariableNames', matlab.lang.makeUniqueStrings(matlab.lang.makeValidName(names)))];
    writetable(Tdyn, fullfile(res_dir, 'dynamic_ml.csv'));
    save(fullfile(res_dir, 'dynamic_ml.mat'), 'Tdyn', 'Mm', 'names', 'lab');
    fprintf('Wrote dynamic_ml.csv\n');
end

%% ---------------- C. AGV missions with ML flux: healthy + 3 faults (RQ4, RQ6) ----------------
if f_done('agv_ml_results.mat')
    load(fullfile(res_dir, 'agv_ml_results.mat'), 'agv_ml');
    fprintf('[C] skipped (agv_ml_results.mat exists)\n');
else
    ck = fullfile(res_dir, 'agv_ml_partial.mat');
    agv_ml = cell(0, 4);
    if exist(ck, 'file'), load(ck, 'agv_ml'); end
    for d = 1:2
        for fault = 0:3
            if ~isempty(agv_ml) && any(strcmp(agv_ml(:,1), D(d).tag) & cell2mat(agv_ml(:,3)) == fault)
                continue;
            end
            P = pick_strategy(D(d), 3);  P.agv.fault_type = fault;
            fprintf('%s  AGV %s ML flux fault %d ...\n', datestr(now, 'HH:MM:SS'), D(d).tag, fault);
            diary off; diary on;
            try
                r = run_agv(P, ENGINE);
            catch err
                fprintf(2, '%s ML fault %d FAILED: %s\n', D(d).tag, fault, err.message);
                r.M = struct('completed', 0, 'E_Wh', NaN, 'e_cross_max_cm', NaN, 'error', err.message);
            end
            agv_ml(end+1, :) = {D(d).tag, 'ML', fault, r.M}; %#ok<SAGROW>
            save(ck, 'agv_ml');
            fprintf('%s ML fault %d: done %d, %.2f Wh, e_max %.1f cm\n', D(d).tag, fault, ...
                r.M.completed, r.M.E_Wh, r.M.e_cross_max_cm);
            clear r
        end
    end
    save(fullfile(res_dir, 'agv_ml_results.mat'), 'agv_ml');
end

% combined AGV table: section 9 (rated, analytic LUT) + section 10 (ML)
A9 = load(fullfile(res_dir, 'agv_results.mat'), 'agv');
fl = {1, 'rated'; 3, 'analytic LUT'};
rows = {};
for k = 1:size(A9.agv, 1)
    rows(end+1, :) = [A9.agv(k,1), fl(cell2mat(fl(:,1)) == A9.agv{k,2}, 2), A9.agv(k,3), {A9.agv{k,4}}]; %#ok<SAGROW>
end
rows = [rows; agv_ml];
fault_names = {'healthy', 'speed sensor', 'current sensor', 'open phase'};
C = cell(size(rows, 1), 10);
for k = 1:size(rows, 1)
    M = rows{k, 4};
    C(k, :) = {rows{k,1}, rows{k,2}, fault_names{rows{k,3} + 1}, getf(M, 'completed', 0), ...
               getf(M, 'mission_time', NaN), getf(M, 'E_Wh', NaN), getf(M, 'e_cross_max_cm', NaN), ...
               getf(M, 'limp', 0), getf(M, 'stopped', 0), getf(M, 'stop_distance_m', NaN)};
end
T = cell2table(C, 'VariableNames', {'drive', 'flux', 'fault', 'completed', 'time_s', 'E_Wh', 'e_max_cm', ...
                                    'limp', 'stopped', 'stop_dist_m'});
T = sortrows(T, {'drive', 'fault', 'flux'});
writetable(T, fullfile(res_dir, 'agv_all_flux.csv'));

%% ---------------- summary ----------------
fprintf('\n==================== A. STEADY-STATE INPUT-POWER SAVING vs RATED FLUX (%%) ====================\n');
for d = 1:2
    E = csvread(fullfile(res_dir, ['efficiency_ml_' D(d).tag '.csv']), 1, 0);
    fprintf('%s (ML = %s)\n   speed  load   |  analytic LUT    ML    | psi_ref LUT / ML (Wb)\n', D(d).tag, D(d).ml_name);
    for a = unique(E(:,1))'
        for b = unique(E(:,2))'
            q = @(s) E(E(:,1) == a & E(:,2) == b & E(:,3) == s, :);
            l = q(2);  m = q(3);
            fprintf('   %3.0f %%  %3.0f %%  |   %6.2f      %6.2f   |   %.3f / %.3f\n', ...
                100*a/D(d).P.m.wm_rated, 100*b/D(d).P.m.T_rated, l(13), m(13), l(4), m(4));
        end
    end
    fprintf('   mean saving: analytic LUT %.2f %%, ML %.2f %%\n', mean(E(E(:,3)==2, 13)), mean(E(E(:,3)==3, 13)));
end
fprintf('\n==================== B. LOAD STEP (dynamic cost) ====================\n');
Tdyn = readtable(fullfile(res_dir, 'dynamic_ml.csv'));
disp(Tdyn(:, 1:min(width(Tdyn), 10)));
fprintf('\n==================== C. AGV MISSIONS (all flux strategies) ====================\n');
disp(T);
fprintf('\nSECTION 10 FINISHED. The tables above are saved in results\\:\n');
fprintf('efficiency_ml_IFOC.csv, efficiency_ml_DTC.csv, dynamic_ml.csv, agv_all_flux.csv.\n');
diary off
end

%% ---------------- local functions ----------------
function P = pick_strategy(Dd, s)
switch s
    case 1, P = Dd.P;    P.ctrl.flux_mode = 1;     % rated
    case 2, P = Dd.P;    P.ctrl.flux_mode = 3;     % analytic loss-model LUT
    case 3, P = Dd.Pml;                            % ML (mode already set)
end
end

function row = eff_row(r, i, key, I)
M = compute_metrics(r);
ix = r.t >= r.scen.win.ss(1);
row = [key(i,:) mean(r.lc(ix, I.c.psi_ref)) M.P_in M.eta_drive M.eta_motor ...
       M.P_fe M.P_cus M.P_cur M.Te_ripple_rms M.thd50];
end

function v = getf(s, f, d)
if isstruct(s) && isfield(s, f) && ~isempty(s.(f)), v = double(s.(f)); else, v = d; end
end
