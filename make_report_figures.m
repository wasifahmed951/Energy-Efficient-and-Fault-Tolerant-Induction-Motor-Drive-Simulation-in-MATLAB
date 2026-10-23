%% make_report_figures.m  -  every thesis figure, saved for report writing
%
%   Run from the "Electrical Drive" folder, after all result files exist:
%       make_report_figures                 all figures (30-60 min: the AGV traces re-run 8 missions)
%       make_report_figures(struct('agv', false))      skip the AGV mission traces (fast)
%       make_report_figures(struct('force', true))     redraw ALL figures (do this after any result changes)
%       make_report_figures(struct('width_cm', 8.8))   IEEE single-column width instead of double column
%       make_report_figures(struct('tiff', true))      also save .tif
%
%   Style: IEEE (Times New Roman, 8 pt text, 1 pt lines), 18.1 cm wide (double column), PNG at 600 dpi.
%
%   Output:  results\figures\<NN_name>.png   (600 dpi, for Word)
%                              <NN_name>.pdf   (vector, for LaTeX)
%                              <NN_name>.fig   (MATLAB figure, can be edited later)
%            results\figures\python\           copies of the notebook figures (python_out\*.png)
%            results\figures\figure_list.txt   what each file shows
%
%   Two kinds of figures:
%     A. drawn from the saved result files (tuning, comparison, flux, efficiency, faults,
%        fairness, ML, AGV energy) - no simulation;
%     B. time plots from short re-runs of the thesis test cycles with the final parameters
%        (results\tuned_params.mat), using exactly the same run_sim / run_agv as the thesis.
%   A figure whose input file is missing is skipped with a message; the others still run.
%   Already saved figures are skipped (resumable), unless opt.force = true.

function make_report_figures(opt)
if nargin < 1, opt = struct(); end
if ~isfield(opt, 'agv'), opt.agv = true; end
if ~isfield(opt, 'force'), opt.force = false; end
if ~isfield(opt, 'engine'), opt.engine = 'simulink'; end
if ~isfield(opt, 'visible'), opt.visible = 'off'; end     % 'on' to watch them appear
if ~isfield(opt, 'width_cm'), opt.width_cm = 18.1; end    % IEEE double column 18.1 cm; 8.8 = single column
if ~isfield(opt, 'dpi'), opt.dpi = 600; end               % IEEE asks >= 600 dpi for line art
if ~isfield(opt, 'tiff'), opt.tiff = false; end           % true: also save .tif (some journals ask for TIFF)
setup_project();
root = fileparts(mfilename('fullpath'));
G.rd  = fullfile(root, 'results');
G.py  = fullfile(root, 'python_out');
G.fd  = fullfile(G.rd, 'figures');
if ~exist(G.fd, 'dir'), mkdir(G.fd); end
G.opt = opt;  G.list = {};
G.col = struct('IFOC', [0 0.447 0.741], 'DTC', [0.850 0.325 0.098], ...
               'rated', [0.5 0.5 0.5], 'LUT', [0.466 0.674 0.188], 'ML', [0.494 0.184 0.556], ...
               'ZN', [0.5 0.5 0.5], 'GA', [0.929 0.694 0.125], 'PSO', [0.301 0.745 0.933]);
% IEEE style: Times, 8 pt text, 1 pt lines, double-column width 18.1 cm (opt.width_cm)
sty = {'defaultAxesFontName', 'Times New Roman'; 'defaultTextFontName', 'Times New Roman'; ...
       'defaultAxesFontSize', 8; 'defaultLegendFontSize', 8; 'defaultLegendFontSizeMode', 'manual'; ...
       'defaultAxesTitleFontSizeMultiplier', 1; 'defaultAxesLabelFontSizeMultiplier', 1; ...
       'defaultLineLineWidth', 1; 'defaultAxesLineWidth', 0.6};
for k = 1:size(sty, 1)
    try, set(groot, sty{k, 1}, sty{k, 2}); catch, end      % skip any setting this MATLAB version lacks
end
t0 = tic;

% ---------------- A. figures from saved results ----------------
G = run_fig(G, @fig_tuning_convergence);
G = run_fig(G, @fig_tuning_spread);
G = run_fig(G, @fig_zn_relay);
G = run_fig(G, @fig_rq12_bars);
G = run_fig(G, @fig_ifoc_vs_dtc_bars);
G = run_fig(G, @fig_flux_sweep_curves);
G = run_fig(G, @fig_loss_breakdown);
G = run_fig(G, @fig_flux_maps);
G = run_fig(G, @fig_ml_vs_lut_map);
G = run_fig(G, @fig_efficiency_ml);
G = run_fig(G, @fig_dynamic_ml);
G = run_fig(G, @fig_fault_study);
G = run_fig(G, @fig_fairness_check2);
G = run_fig(G, @fig_agv_energy);
G = run_fig(G, @copy_python_figures);
% ---------------- B. time plots from short re-runs ----------------
G = run_fig(G, @fig_sim_step);
G = run_fig(G, @fig_sim_ripple_zoom);
G = run_fig(G, @fig_sim_current);
G = run_fig(G, @fig_sim_load_step);
G = run_fig(G, @fig_sim_reversal);
G = run_fig(G, @fig_sim_zn_vs_pso);
G = run_fig(G, @fig_sim_faults);
G = run_fig(G, @fig_sim_flux_strategies);
if opt.agv, G = run_fig(G, @fig_agv_traces); end

% ---------------- list ----------------
fid = fopen(fullfile(G.fd, 'figure_list.txt'), 'w');
fprintf(fid, 'Thesis figures (make_report_figures, %s)\n\n', datestr(now, 'yyyy-mm-dd HH:MM'));
for k = 1:size(G.list, 1), fprintf(fid, '%-34s %s\n', G.list{k, 1}, G.list{k, 2}); end
fclose(fid);
fprintf('\nDone in %.0f s: %d figures in %s\n', toc(t0), size(G.list, 1), G.fd);
end

% =====================================================================
%  helpers
% =====================================================================
function G = run_fig(G, fn)
try
    G = fn(G);
catch err
    fprintf(2, '  %s skipped: %s\n', func2str(fn), err.message);
end
end

function [f, skip] = newfig(G, name, w, h)
% w, h in cm (default 16 x 10, a Word page width)
if nargin < 3, w = 16; end
if nargin < 4, h = 10; end
sc = G.opt.width_cm/16;  w = w*sc;  h = h*sc;      % all layouts are drawn for 16 cm, scaled to the column width
skip = ~G.opt.force && exist(fullfile(G.fd, [name '.png']), 'file');
f = [];
if skip, fprintf('  %s exists (skipped)\n', name); return; end
f = figure('Color', 'w', 'Units', 'centimeters', 'Position', [2 2 w h], 'Visible', G.opt.visible, ...
           'PaperPositionMode', 'auto');
end

function G = savefig_all(G, f, name, caption)
p = fullfile(G.fd, name);
exportgraphics(f, [p '.png'], 'Resolution', G.opt.dpi);
if G.opt.tiff, exportgraphics(f, [p '.tif'], 'Resolution', G.opt.dpi); end
exportgraphics(f, [p '.pdf'], 'ContentType', 'vector');
set(f, 'Visible', 'on');  savefig(f, [p '.fig']);  close(f);
G.list(end+1, :) = {name, caption};
fprintf('  saved %s\n', name);
end

function G = note(G, name, caption)
% figure skipped because it exists: keep it in the list
G.list(end+1, :) = {name, caption};
end

function T = rtab(G, f)
p = fullfile(G.rd, f);
if ~exist(p, 'file'), p = fullfile(G.py, f); end
if ~exist(p, 'file'), error('%s not found', f); end
T = readtable(p, 'TextType', 'char', 'VariableNamingRule', 'preserve');
end

function S = rmat(G, f, varargin)
p = fullfile(G.rd, f);
if ~exist(p, 'file'), error('%s not found', f); end
S = load(p, varargin{:});
end

function [Pi, Pd, Pi_zn, Pd_zn] = params(G)
S = rmat(G, 'tuned_params.mat', 'Pi_best', 'Pd_best', 'Pi_zn', 'Pd_zn');
Pi = S.Pi_best;  Pd = S.Pd_best;  Pi_zn = S.Pi_zn;  Pd_zn = S.Pd_zn;
end

function [Pi_ml, Pd_ml] = ml_params(G, Pi, Pd)
% the deployed ML flux models, exactly as run_ml_flux_all builds them
Pi_ml = build_flux_lut(Pi, 'csv', fullfile(G.py, 'lut_kan_IFOC.csv'));
Pi_ml.ctrl.flux_mode = 3;
Mw = load(fullfile(G.py, 'mlp_DTC.mat'));
Pd_ml = Pd;
for f = fieldnames(Pd.mlp)'
    Pd_ml.mlp.(f{1}) = double(Mw.mlp.(f{1}));
end
Pd_ml.ctrl.flux_mode = 4;
end

function r = sim(G, P, name, sopt)
if nargin < 4, sopt = struct(); end
r = run_sim(P, make_scenario(name, P, sopt), G.opt.engine);
end

function c = colof(G, key)
key = strrep(strrep(key, 'analytic LUT', 'LUT'), ' ', '');
if isfield(G.col, key), c = G.col.(key); else, c = [0 0 0]; end
end

function J = corrected_J(G, drive, method, seed, Jold)
% corrected J from rescore_tuning (if it was run), else the stored J
J = Jold;
p = fullfile(G.rd, 'tuning_rescored.mat');
if ~exist(p, 'file'), return; end
S = load(p, 'T');  T = S.T;
k = find(strcmp(T.drive, drive) & strcmpi(T.method, method) & T.seed == seed, 1);
if ~isempty(k), J = T.J_new(k); end
end

function J = zn_J(G, drive)
J = NaN;
p = fullfile(G.rd, 'tuning_rescored.mat');
if ~exist(p, 'file'), return; end
S = load(p, 'T');  T = S.T;
k = find(strcmp(T.drive, drive) & strcmp(T.method, 'ZN'), 1);
if ~isempty(k), J = T.J_new(k); end
end

% =====================================================================
%  A. figures from saved results
% =====================================================================
function G = fig_tuning_convergence(G)
name = '01_tuning_convergence';  cap = 'GA vs PSO convergence (best J vs evaluations), 5 seeds, both drives';
[f, skip] = newfig(G, name, 16, 7);  if skip, G = note(G, name, cap); return; end
U = rmat(G, 'tuning_runs.mat', 'tun');  tun = U.tun;
resc = exist(fullfile(G.rd, 'tuning_rescored.mat'), 'file');
drives = {'IFOC', 'DTC'};
for d = 1:2
    subplot(1, 2, d);  hold on;  h = gobjects(0);  lab = {};
    for k = 1:numel(tun)
        if ~strcmp(tun(k).drive, drives{d}), continue; end
        o = tun(k).out;  m = upper(tun(k).method);
        s = corrected_J(G, drives{d}, m, o.seed, o.J)/o.J;      % rescale the history to the corrected J
        hh = plot(o.hist(:, 1), o.hist(:, 2)*s, 'Color', colof(G, m));
        if ~any(strcmp(lab, m)), h(end+1) = hh; lab{end+1} = m; end %#ok<AGROW>
    end
    Jz = zn_J(G, drives{d});
    if isfinite(Jz), h(end+1) = yline(Jz, '--', 'Color', colof(G, 'ZN')); lab{end+1} = 'ZN'; end
    set(gca, 'YScale', 'log');
    grid on;  box on;  xlabel('function evaluations');  ylabel('best J (log scale)');  title(drives{d});
    legend(h, lab, 'Location', 'northeast');
end
if resc, cap = [cap ' (histories scaled to the corrected J)']; end
G = savefig_all(G, f, name, cap);
end

function G = fig_tuning_spread(G)
name = '02_tuning_J_per_seed';  cap = 'Final J of every GA and PSO seed, with the ZN baseline';
[f, skip] = newfig(G, name, 16, 7);  if skip, G = note(G, name, cap); return; end
U = rmat(G, 'tuning_runs.mat', 'tun');  tun = U.tun;
drives = {'IFOC', 'DTC'};  meth = {'GA', 'PSO'};
for d = 1:2
    subplot(1, 2, d);  hold on;
    for m = 1:2
        J = [];
        for k = 1:numel(tun)
            if strcmp(tun(k).drive, drives{d}) && strcmpi(tun(k).method, meth{m})
                J(end+1) = corrected_J(G, drives{d}, meth{m}, tun(k).out.seed, tun(k).out.J); %#ok<AGROW>
            end
        end
        x = m + 0.08*(1:numel(J)) - 0.24;
        plot(x, J, 'o', 'MarkerFaceColor', colof(G, meth{m}), 'MarkerEdgeColor', 'k', 'MarkerSize', 7);
        plot([m-0.3 m+0.3], [1 1]*mean(J), 'k-', 'LineWidth', 1.5);
    end
    Jz = zn_J(G, drives{d});
    if isfinite(Jz), yline(Jz, '--', 'ZN', 'Color', colof(G, 'ZN')); end
    set(gca, 'XTick', 1:2, 'XTickLabel', meth);  xlim([0.4 2.6]);
    grid on;  box on;  ylabel('J (corrected)');  title(drives{d});
end
G = savefig_all(G, f, name, cap);
end

function G = fig_zn_relay(G)
name = '03_zn_relay_test';  cap = 'Relay (Ziegler-Nichols) test: sustained speed oscillation used for Ku, Pu';
[f, skip] = newfig(G, name, 16, 7);  if skip, G = note(G, name, cap); return; end
S = rmat(G, 'zn_params.mat', 'zn_ifoc', 'zn_dtc');
Z = {S.zn_ifoc, S.zn_dtc};  dn = {'IFOC', 'DTC'};  I = log_index();
for d = 1:2
    subplot(1, 2, d);  r = Z{d}.res;
    plot(r.t, r.lp(:, I.p.w)*30/pi, 'Color', colof(G, dn{d}));
    grid on;  xlabel('time [s]');  ylabel('speed [rpm]');
    title(sprintf('%s: a = %.3g rad/s, P_u = %.2f ms', dn{d}, Z{d}.a, 1e3*Z{d}.Pu));
    xlim([max(r.t(1), r.t(end) - 0.3) r.t(end)]);
end
G = savefig_all(G, f, name, cap);
end

function G = fig_rq12_bars(G)
S = rmat(G, 'comparison_rq1_rq2.mat', 'rows', 'Mm', 'names');
names = cellstr(S.names);  rows = S.rows;
cyc = {'step_rated', 'load_step', 'reversal'};
met = {'overshoot', '%';  'settling_time', 's';  'itae', '';  'Te_ripple_pp', '%T_r';  ...
       'Te_ripple_rms', '%T_r';  'thd_total', '%';  'load_dip_rpm', 'rpm';  'eta_drive', ''};
grp = {'IFOC', 'ZN'; 'IFOC', 'GA'; 'IFOC', 'PSO'; 'DTC', 'ZN'; 'DTC', 'GA'; 'DTC', 'PSO'};
for c = 1:3
    name = sprintf('04_rq12_%s', cyc{c});
    cap = sprintf('ZN / GA / PSO x IFOC / DTC on %s (RQ1, RQ2)', strrep(cyc{c}, '_', ' '));
    [f, skip] = newfig(G, name, 16, 14);  if skip, G = note(G, name, cap); continue; end
    sel = met(cellfun(@(m) any(isfinite(col(S.Mm, names, m, rows, cyc{c}, grp))), met(:, 1)), :);
    nm = size(sel, 1);  nc = 2;  nr = ceil(nm/nc);
    for k = 1:nm
        subplot(nr, nc, k);
        v = col(S.Mm, names, sel{k, 1}, rows, cyc{c}, grp);
        b = bar(reshape(v, 3, 2)', 'grouped');
        for j = 1:3, set(b(j), 'FaceColor', colof(G, grp{j, 2})); end
        set(gca, 'XTickLabel', {'IFOC', 'DTC'});  grid on;
        ylabel(sel{k, 2});  title(strrep(sel{k, 1}, '_', ' '));
        if k == 1, legend({'ZN', 'GA', 'PSO'}, 'Location', 'best'); end
    end
    G = savefig_all(G, f, name, cap);
end
end

function v = col(Mm, names, metric, rows, cycle, grp)
j = find(strcmp(names, metric), 1);
v = NaN(size(grp, 1), 1);
if isempty(j), return; end
for g = 1:size(grp, 1)
    k = find(strcmp(rows(:, 1), grp{g, 1}) & strcmp(rows(:, 2), grp{g, 2}) & strcmp(rows(:, 3), cycle), 1);
    if ~isempty(k), v(g) = Mm(k, j); end
end
end

function G = fig_ifoc_vs_dtc_bars(G)
name = '05_ifoc_vs_dtc_quality';  cap = 'IFOC vs DTC with PSO gains: torque ripple and current THD';
[f, skip] = newfig(G, name, 16, 9);  if skip, G = note(G, name, cap); return; end
S = rmat(G, 'comparison_rq1_rq2.mat', 'rows', 'Mm', 'names');
names = cellstr(S.names);
cyc = {'step_rated', 'load_step', 'reversal'};
met = {'Te_ripple_pp', 'torque ripple p-p [%T_r]'; 'Te_ripple_rms', 'torque ripple rms [%T_r]'; ...
       'thd50', 'current THD_{50} [%]'; 'thd_total', 'current THD total [%]'};
for k = 1:4
    subplot(2, 2, k);
    v = zeros(3, 2);
    for c = 1:3
        v(c, :) = col(S.Mm, names, met{k, 1}, S.rows, cyc{c}, {'IFOC', 'PSO'; 'DTC', 'PSO'})';
    end
    b = bar(v, 'grouped');  set(b(1), 'FaceColor', G.col.IFOC);  set(b(2), 'FaceColor', G.col.DTC);
    set(gca, 'XTickLabel', {'step', 'load step', 'reversal'});  grid on;  ylabel(met{k, 2});
    if k == 1, legend({'IFOC', 'DTC'}, 'Location', 'best'); end
end
G = savefig_all(G, f, name, cap);
end

function G = fig_flux_sweep_curves(G)
name = '06_flux_sweep_Pin_vs_psi';  cap = 'Input power vs rotor-flux reference at three loads (flux sweep), optimum marked';
[f, skip] = newfig(G, name, 16, 7);  if skip, G = note(G, name, cap); return; end
dn = {'IFOC', 'DTC'};
for d = 1:2
    T = rtab(G, ['flux_sweep_' dn{d} '.csv']);
    if ismember('ok', T.Properties.VariableNames), T = T(T.ok > 0, :); end
    wv = unique(T.w);  Tv = unique(T.T);
    w0 = wv(round(numel(wv)/2));                       % mid speed
    pick = Tv(unique(round(linspace(1, max(1, round(numel(Tv)*0.6)), 3))));
    subplot(1, 2, d);  hold on;  lab = {};
    cm = lines(numel(pick));
    for j = 1:numel(pick)
        s = T(abs(T.w - w0) < 1e-6 & abs(T.T - pick(j)) < 1e-6, :);
        [psi, o] = sort(s.psi);  P = s.P_in(o);
        plot(psi, P, '-o', 'Color', cm(j, :), 'MarkerSize', 3);
        [pm, i] = min(P);  plot(psi(i), pm, 'p', 'Color', cm(j, :), 'MarkerFaceColor', cm(j, :), 'MarkerSize', 10, 'HandleVisibility', 'off');
        lab{end+1} = sprintf('T = %.1f N m', pick(j)); %#ok<AGROW>
    end
    grid on;  box on;  xlabel('\psi_r reference [Wb]');  ylabel('P_{in} [W]');
    title(sprintf('%s, %.0f rpm', dn{d}, w0*30/pi));  legend(lab, 'Location', 'best');
end
G = savefig_all(G, f, name, cap);
end

function G = fig_loss_breakdown(G)
name = '07_loss_breakdown_vs_psi';  cap = 'Loss components vs flux at a light-load point: iron loss falls, copper loss rises';
[f, skip] = newfig(G, name, 16, 7);  if skip, G = note(G, name, cap); return; end
dn = {'IFOC', 'DTC'};
for d = 1:2
    T = rtab(G, ['flux_sweep_' dn{d} '.csv']);
    if ismember('ok', T.Properties.VariableNames), T = T(T.ok > 0, :); end
    wv = unique(T.w);  Tv = unique(T.T);
    w0 = wv(round(numel(wv)/2));  T0 = Tv(min(2, numel(Tv)));
    s = T(abs(T.w - w0) < 1e-6 & abs(T.T - T0) < 1e-6, :);
    [psi, o] = sort(s.psi);  s = s(o, :);
    subplot(1, 2, d);
    plot(psi, s.P_fe, psi, s.P_cus, psi, s.P_cur, psi, s.P_cond + s.P_sw, ...
         psi, s.P_fe + s.P_cus + s.P_cur + s.P_cond + s.P_sw, 'k-');
    grid on;  xlabel('\psi_r reference [Wb]');  ylabel('loss [W]');
    title(sprintf('%s, %.0f rpm, %.1f N m', dn{d}, w0*30/pi, T0));
    legend({'iron', 'stator Cu', 'rotor Cu', 'inverter', 'total'}, 'Location', 'best');
end
G = savefig_all(G, f, name, cap);
end

function G = fig_flux_maps(G)
name = '08_optimal_flux_map';  cap = 'Loss-minimising flux and saving vs rated flux over the operating range';
[f, skip] = newfig(G, name, 16, 12);  if skip, G = note(G, name, cap); return; end
dn = {'IFOC', 'DTC'};
for d = 1:2
    T = rtab(G, ['flux_opt_' dn{d} '.csv']);
    wv = unique(T.w);  Tv = unique(T.T);
    Z = NaN(numel(Tv), numel(wv));  Sv = Z;
    for k = 1:height(T)
        i = find(abs(Tv - T.T(k)) < 1e-9, 1);  j = find(abs(wv - T.w(k)) < 1e-9, 1);
        Z(i, j) = T.psi_opt(k);  Sv(i, j) = T.saving_pct(k);
    end
    subplot(2, 2, d);
    contourf(wv*30/pi, Tv, Z, 12);  colorbar;  xlabel('speed [rpm]');  ylabel('torque [N m]');
    title([dn{d} ': optimal \psi_r [Wb]']);
    subplot(2, 2, 2 + d);
    contourf(wv*30/pi, Tv, Sv, 12);  colorbar;  xlabel('speed [rpm]');  ylabel('torque [N m]');
    title([dn{d} ': saving vs rated flux [%]']);
end
G = savefig_all(G, f, name, cap);
end

function G = fig_ml_vs_lut_map(G)
name = '09_ml_vs_analytic_flux';  cap = 'Deployed ML flux model vs analytic loss-model LUT on the controller grid';
[f, skip] = newfig(G, name, 16, 12);  if skip, G = note(G, name, cap); return; end
[Pi, Pd] = params(G);
[Pi_ml, Pd_ml] = ml_params(G, Pi, Pd);
D = {Pi, Pi_ml, 'IFOC (KAN)'; Pd, Pd_ml, 'DTC (MLP)'};
for d = 1:2
    P = D{d, 1};  wg = P.lut.w;  Tg = P.lut.T;
    A = P.lut.psi;
    if d == 1
        B = D{d, 2}.lut.psi;
    else
        B = NaN(size(A));
        for i = 1:numel(wg)
            for j = 1:numel(Tg)
                B(i, j) = mlp_eval(D{d, 2}.mlp, wg(i), Tg(j), P);
            end
        end
    end
    if ~isequal(size(A), [numel(wg) numel(Tg)]), A = A'; B = B'; end
    subplot(2, 2, d);
    contourf(wg*30/pi, Tg, A', 12);  colorbar;  title([D{d, 3} ': analytic LUT \psi_r [Wb]']);
    xlabel('speed [rpm]');  ylabel('torque [N m]');
    subplot(2, 2, 2 + d);
    contourf(wg*30/pi, Tg, 1e3*(B - A)', 12);  colorbar;  title('ML - analytic [mWb]');
    xlabel('speed [rpm]');  ylabel('torque [N m]');
end
G = savefig_all(G, f, name, cap);
end

function y = mlp_eval(m, w, T, P)
% same MLP as the controller (ctrl_step.m, flux_target, flux_mode 4), clipped like the controller
xn = ([w; T] - m.x_mu(:))./m.x_sd(:);
y = m.W2*tanh(m.W1*xn + m.b1(:)) + m.b2;
y = y*m.y_sd + m.y_mu;
y = min(max(y, P.ctrl.psi_min), P.m.psi_r_rated);
end

function G = fig_efficiency_ml(G)
name = '10_efficiency_rated_lut_ml';  cap = 'Drive efficiency at light load: rated flux vs analytic LUT vs ML flux (RQ3)';
[f, skip] = newfig(G, name, 16, 12);  if skip, G = note(G, name, cap); return; end
dn = {'IFOC', 'DTC'};  st = {'rated', 'LUT', 'ML'};
for d = 1:2
    T = rtab(G, ['efficiency_ml_' dn{d} '.csv']);
    wv = unique(T.w);  Tv = unique(T.T);
    lab = {};  E = [];  Sv = [];
    for i = 1:numel(Tv)
        for j = 1:numel(wv)
            r = T(abs(T.w - wv(j)) < 1e-6 & abs(T.T - Tv(i)) < 1e-6, :);
            e = NaN(1, 3);  s = e;
            for k = 1:3
                q = find(r.strategy == k, 1);
                if ~isempty(q), e(k) = r.eta_drive(q); s(k) = r.saving_pct(q); end
            end
            E(end+1, :) = e;  Sv(end+1, :) = s; %#ok<AGROW>
            lab{end+1} = sprintf('%.0f rpm\n%.1f N m', wv(j)*30/pi, Tv(i)); %#ok<AGROW>
        end
    end
    subplot(2, 1, d);
    b = bar(E, 'grouped');
    for k = 1:3, set(b(k), 'FaceColor', colof(G, st{k})); end
    set(gca, 'XTick', 1:numel(lab), 'XTickLabel', strrep(lab, newline, ' / '), 'XTickLabelRotation', 30);
    ylabel('\eta_{drive}');  title(dn{d});  grid on;  ylim([0.4 0.9]);
    if d == 1, legend({'rated flux', 'analytic LUT', 'ML'}, 'Location', 'southeast'); end
end
G = savefig_all(G, f, name, cap);
end

function G = fig_dynamic_ml(G)
name = '11_flux_dynamic_cost';  cap = 'Dynamic cost of flux optimisation on the load step: ripple, ITAE, speed dip';
[f, skip] = newfig(G, name, 16, 9);  if skip, G = note(G, name, cap); return; end
T = rtab(G, 'dynamic_ml.csv');
met = {'Te_ripple_pp', 'ripple p-p [%T_r]'; 'Te_ripple_rms', 'ripple rms [%T_r]'; ...
       'itae', 'ITAE'; 'load_dip_rpm', 'speed dip [rpm]'};
fl = {'rated', 'analytic LUT', 'ML'};  dn = {'IFOC', 'DTC'};
for k = 1:4
    subplot(2, 2, k);  v = NaN(2, 3);
    for d = 1:2
        for j = 1:3
            q = find(strcmp(T.drive, dn{d}) & strcmp(T.flux, fl{j}), 1);
            if ~isempty(q), v(d, j) = T.(met{k, 1})(q); end
        end
    end
    b = bar(v, 'grouped');
    for j = 1:3, set(b(j), 'FaceColor', colof(G, fl{j})); end
    set(gca, 'XTickLabel', dn);  ylabel(met{k, 2});  grid on;
    if k == 1, legend({'rated', 'LUT', 'ML'}, 'Location', 'best'); end
end
G = savefig_all(G, f, name, cap);
end

function G = fig_fault_study(G)
name = '12_fault_study';  cap = 'Fault study: detection time, torque ripple before/after, speed error (RQ4)';
[f, skip] = newfig(G, name, 16, 18);  if skip, G = note(G, name, cap); return; end
p = fullfile(G.rd, 'fault_study.csv');
if ~exist(p, 'file'), error('fault_study.csv not found'); end
A = csvread(p, 1, 0);       % header has commas inside its names, so read by column number
% columns: 1 drive 2 fault 3 flux_mode 4 remedy 5 detected 6 t_detect_ms 7 pre p-p 8 post p-p 9 speed err
cd_ = A(:, 1);  cf = A(:, 2);  cm = A(:, 3);  cr = A(:, 4);
fn = {'speed', 'current', 'open'};  dn = {'IFOC', 'DTC'};
lab = {};  td = [];  pre = [];  post = [];  err = [];
for k = 1:size(A, 1)
    s = sprintf('%s %s %s', dn{cd_(k)}, fn{cf(k)}, ternary(cm(k) == 1, 'rated', 'LUT'));
    if cf(k) == 3 && cd_(k) == 1, s = [s ternary(cr(k) == 1, ' +neutral', ' no remedy')]; end
    if cf(k) == 3 && cd_(k) == 2, s = [s ' derate']; end
    lab{end+1} = s; %#ok<AGROW>
    td(end+1) = A(k, 6);  pre(end+1) = A(k, 7);  post(end+1) = A(k, 8);  err(end+1) = A(k, 9); %#ok<AGROW>
end
x = 1:numel(lab);
subplot(3, 1, 1);  b = bar(x, td);  set(b, 'FaceColor', [0.3 0.3 0.3]);
ylabel('t_{detect} [ms]');  grid on;  set(gca, 'XTick', x, 'XTickLabel', {});
subplot(3, 1, 2);  b = bar(x, [pre(:) post(:)], 'grouped');  set(b(1), 'FaceColor', [0.6 0.6 0.6]);  set(b(2), 'FaceColor', G.col.DTC);
set(gca, 'YScale', 'log');  ylabel('ripple p-p [%T_r]');  legend({'pre-fault', 'post-fault'}, 'Location', 'northwest');
grid on;  set(gca, 'XTick', x, 'XTickLabel', {});
subplot(3, 1, 3);  b = bar(x, max(err, 1e-3));  set(b, 'FaceColor', G.col.IFOC);
set(gca, 'YScale', 'log', 'XTick', x, 'XTickLabel', lab, 'XTickLabelRotation', 60, 'FontSize', 8);
ylabel('post-fault speed error [%]');  grid on;
G = savefig_all(G, f, name, cap);
end

function G = fig_fairness_check2(G)
name = '13_fairness_dtc_at_5kHz';  cap = 'Fairness CHECK 2: DTC with bands re-calibrated to 5 kHz vs IFOC';
[f, skip] = newfig(G, name, 16, 9);  if skip, G = note(G, name, cap); return; end
T = rtab(G, 'table_fsw_matched.csv');
cyc = {'step_rated', 'load_step', 'reversal'};
drv = {'IFOC', 'DTC (section-2 bands)', 'DTC (matched 5 kHz)'};
met = {'Te_ripple_pp', 'ripple p-p [%T_r]'; 'Te_ripple_rms', 'ripple rms [%T_r]'; ...
       'thd_total', 'THD total [%]'; 'fsw_Hz', 'f_{sw} [Hz]'};
cc = [G.col.IFOC; G.col.DTC; 0.6 0.15 0.05];
for k = 1:4
    subplot(2, 2, k);  v = NaN(3, 3);
    for c = 1:3
        for j = 1:3
            q = find(strcmp(T.cycle, cyc{c}) & strcmp(T.drive, drv{j}), 1);
            if ~isempty(q), v(c, j) = T.(met{k, 1})(q); end
        end
    end
    b = bar(v, 'grouped');
    for j = 1:3, set(b(j), 'FaceColor', cc(j, :)); end
    set(gca, 'XTickLabel', {'step', 'load', 'reversal'});  ylabel(met{k, 2});  grid on;
    if k == 1, legend({'IFOC', 'DTC (own bands)', 'DTC (5 kHz)'}, 'Location', 'best', 'FontSize', 8); end
end
G = savefig_all(G, f, name, cap);
end

function G = fig_agv_energy(G)
name = '14_agv_mission_energy';  cap = 'AGV mission energy per drive, flux strategy and fault (RQ6)';
[f, skip] = newfig(G, name, 16, 9);  if skip, G = note(G, name, cap); return; end
T = rtab(G, 'agv_all_flux.csv');
fa = {'healthy', 'speed sensor', 'current sensor', 'open phase'};
fl = {'rated', 'analytic LUT', 'ML'};  dn = {'IFOC', 'DTC'};
for d = 1:2
    subplot(1, 2, d);  v = NaN(4, 3);  done = true(4, 3);
    for i = 1:4
        for j = 1:3
            q = find(strcmp(T.drive, dn{d}) & strcmp(T.flux, fl{j}) & strcmp(T.fault, fa{i}), 1);
            if ~isempty(q), v(i, j) = T.E_Wh(q); done(i, j) = T.completed(q) > 0; end
        end
    end
    b = bar(v, 'grouped');
    for j = 1:3, set(b(j), 'FaceColor', colof(G, fl{j})); end
    set(gca, 'XTickLabel', {'healthy', 'speed s.', 'current s.', 'open ph.'});
    ylabel('mission energy [Wh]');  title(dn{d});  grid on;
    if any(~done(:)), text(4, max(v(:))*0.95, 'not completed (safe stop)', 'HorizontalAlignment', 'center', 'FontSize', 8); end
    if d == 1, legend({'rated', 'LUT', 'ML'}, 'Location', 'northwest'); end
end
G = savefig_all(G, f, name, cap);
end

function G = copy_python_figures(G)
d = fullfile(G.fd, 'python');
if ~exist(d, 'dir'), mkdir(d); end
L = dir(fullfile(G.py, '*.png'));
if isempty(L), error('no PNG files in python_out (run the notebooks first)'); end
for k = 1:numel(L)
    copyfile(fullfile(L(k).folder, L(k).name), fullfile(d, L(k).name));
    G.list(end+1, :) = {['python\' L(k).name], 'notebook figure (copied from python_out)'};
end
fprintf('  copied %d notebook figures to %s\n', numel(L), d);
end

% =====================================================================
%  B. time plots from short re-runs (final parameters)
% =====================================================================
function G = fig_sim_step(G)
name = '20_sim_speed_step';  cap = 'Rated speed step with 50 % load: IFOC vs DTC (PSO gains)';
[f, skip] = newfig(G, name, 16, 13);  if skip, G = note(G, name, cap); return; end
[Pi, Pd] = params(G);  I = log_index();
R = {sim(G, Pi, 'step_rated'), sim(G, Pd, 'step_rated')};  dn = {'IFOC', 'DTC'};
subplot(3, 1, 1);  hold on;
plot(R{1}.t, R{1}.lc(:, I.c.w_ref)*30/pi, 'k--');
for d = 1:2, plot(R{d}.t, R{d}.lp(:, I.p.w)*30/pi, 'Color', G.col.(dn{d})); end
ylabel('speed [rpm]');  legend({'reference', 'IFOC', 'DTC'}, 'Location', 'southeast');  grid on;
subplot(3, 1, 2);  hold on;
for d = 1:2, plot(R{d}.t, R{d}.lp(:, I.p.Te), 'Color', G.col.(dn{d})); end
ylabel('T_e [N m]');  grid on;
subplot(3, 1, 3);  hold on;
for d = 1:2, plot(R{d}.t, R{d}.lp(:, I.p.psir), 'Color', G.col.(dn{d})); end
ylabel('|\psi_r| [Wb]');  xlabel('time [s]');  grid on;
G = savefig_all(G, f, name, cap);
end

function G = fig_sim_ripple_zoom(G)
name = '21_sim_torque_ripple_zoom';  cap = 'Steady-state torque ripple (min-max band within each control step), 20 ms window';
[f, skip] = newfig(G, name, 16, 9);  if skip, G = note(G, name, cap); return; end
[Pi, Pd] = params(G);  I = log_index();
R = {sim(G, Pi, 'step_rated'), sim(G, Pd, 'step_rated')};  dn = {'IFOC', 'DTC'};
for d = 1:2
    subplot(2, 1, d);  r = R{d};  ix = r.t >= 0.88 & r.t <= 0.90;  t = r.t(ix)*1e3;
    fill([t; flipud(t)], [r.lp(ix, I.p.Te_min); flipud(r.lp(ix, I.p.Te_max))], G.col.(dn{d}), ...
         'FaceAlpha', 0.3, 'EdgeColor', 'none');  hold on;
    plot(t, r.lp(ix, I.p.Te), 'Color', G.col.(dn{d}));
    ylabel('T_e [N m]');  title(dn{d});  grid on;
    if d == 2, xlabel('time [ms]'); end
    legend({'min-max', 'step average'}, 'Location', 'northeast');
end
G = savefig_all(G, f, name, cap);
end

function G = fig_sim_current(G)
name = '22_sim_phase_current';  cap = 'Phase-a current in steady state and its spectrum (sampled at the control rate)';
[f, skip] = newfig(G, name, 16, 10);  if skip, G = note(G, name, cap); return; end
[Pi, Pd] = params(G);  I = log_index();
R = {sim(G, Pi, 'step_rated'), sim(G, Pd, 'step_rated')};  dn = {'IFOC', 'DTC'};
for d = 1:2
    r = R{d};  ix = r.t >= 0.8 & r.t <= 1.0;  ia = r.lp(ix, I.p.ia);  t = r.t(ix);
    subplot(2, 2, d);  plot((t - t(1))*1e3, ia, 'Color', G.col.(dn{d}));
    xlim([0 60]);  grid on;  xlabel('time [ms]');  ylabel('i_a [A]');  title(dn{d});
    Ts = mean(diff(t));  n = numel(ia);  wn = 0.5 - 0.5*cos(2*pi*(0:n-1)'/(n-1));          % Hann window
    Y = abs(fft((ia - mean(ia)).*wn))/n*4;
    fr = (0:n-1)'/(n*Ts);  k = fr <= min(2500, 0.5/Ts);
    subplot(2, 2, 2 + d);  semilogy(fr(k), max(Y(k), 1e-5), 'Color', G.col.(dn{d}));
    grid on;  xlabel('frequency [Hz]');  ylabel('|I_a| [A]');  ylim([1e-4 20]);
end
G = savefig_all(G, f, name, cap);
end

function G = fig_sim_load_step(G)
name = '23_sim_load_step';  cap = 'Load step 25 % -> 100 % at rated speed: speed dip and torque';
[f, skip] = newfig(G, name, 16, 10);  if skip, G = note(G, name, cap); return; end
[Pi, Pd] = params(G);  I = log_index();
R = {sim(G, Pi, 'load_step'), sim(G, Pd, 'load_step')};  dn = {'IFOC', 'DTC'};
subplot(2, 1, 1);  hold on;
for d = 1:2, plot(R{d}.t, R{d}.lp(:, I.p.w)*30/pi, 'Color', G.col.(dn{d})); end
xlim([0.75 1.0]);  ylabel('speed [rpm]');  legend(dn, 'Location', 'southeast');  grid on;
subplot(2, 1, 2);  hold on;
plot(R{1}.t, R{1}.lp(:, I.p.TL), 'k--');
for d = 1:2, plot(R{d}.t, R{d}.lp(:, I.p.Te), 'Color', G.col.(dn{d})); end
xlim([0.75 1.0]);  ylabel('torque [N m]');  xlabel('time [s]');  legend({'load', 'IFOC', 'DTC'}, 'Location', 'southeast');  grid on;
G = savefig_all(G, f, name, cap);
end

function G = fig_sim_reversal(G)
name = '24_sim_speed_reversal';  cap = 'Speed reversal +50 % -> -50 % rated: speed, torque, phase current';
[f, skip] = newfig(G, name, 16, 13);  if skip, G = note(G, name, cap); return; end
[Pi, Pd] = params(G);  I = log_index();
R = {sim(G, Pi, 'reversal'), sim(G, Pd, 'reversal')};  dn = {'IFOC', 'DTC'};
subplot(3, 1, 1);  hold on;
plot(R{1}.t, R{1}.lc(:, I.c.w_ref)*30/pi, 'k--');
for d = 1:2, plot(R{d}.t, R{d}.lp(:, I.p.w)*30/pi, 'Color', G.col.(dn{d})); end
ylabel('speed [rpm]');  legend({'reference', 'IFOC', 'DTC'}, 'Location', 'northeast');  grid on;
subplot(3, 1, 2);  hold on;
for d = 1:2, plot(R{d}.t, R{d}.lp(:, I.p.Te), 'Color', G.col.(dn{d})); end
ylabel('T_e [N m]');  grid on;
subplot(3, 1, 3);  hold on;
for d = 1:2, plot(R{d}.t, R{d}.lp(:, I.p.ia), 'Color', G.col.(dn{d})); end
ylabel('i_a [A]');  xlabel('time [s]');  grid on;
G = savefig_all(G, f, name, cap);
end

function G = fig_sim_zn_vs_pso(G)
name = '25_sim_zn_ga_pso_step';  cap = 'Rated speed step with ZN, GA and PSO gains (overshoot and settling)';
[f, skip] = newfig(G, name, 16, 7);  if skip, G = note(G, name, cap); return; end
[~, ~, Pi_zn, Pd_zn] = params(G);  I = log_index();
U = rmat(G, 'tuning_runs.mat', 'tun');  tun = U.tun;
dn = {'IFOC', 'DTC'};  Pz = {Pi_zn, Pd_zn};  meth = {'ZN', 'GA', 'PSO'};
for d = 1:2
    subplot(1, 2, d);  hold on;
    for m = 1:3
        P = Pz{d};
        if m > 1
            best = Inf;  x = [];
            for k = 1:numel(tun)
                if strcmp(tun(k).drive, dn{d}) && strcmpi(tun(k).method, meth{m}) && tun(k).out.J < best
                    best = tun(k).out.J;  x = tun(k).out.x;
                end
            end
            P = apply_gains(P, x);
        end
        r = sim(G, P, 'step_rated');
        if m == 1, plot(r.t, r.lc(:, I.c.w_ref)*30/pi, 'k--', 'HandleVisibility', 'off'); end
        plot(r.t, r.lp(:, I.p.w)*30/pi, 'Color', colof(G, meth{m}));
    end
    xlim([0.15 0.5]);  grid on;  xlabel('time [s]');  ylabel('speed [rpm]');  title(dn{d});
    legend(meth, 'Location', 'southeast');
end
G = savefig_all(G, f, name, cap);
end

function G = fig_sim_faults(G)
[Pi, Pd] = params(G);  I = log_index();
cases = {'IFOC', 'speed', 1; 'IFOC', 'current', 1; 'IFOC', 'open', 1; 'IFOC', 'open', 0; ...
         'DTC', 'speed', 1; 'DTC', 'current', 1; 'DTC', 'open', 1};
for c = 1:size(cases, 1)
    dn = cases{c, 1};  ft = cases{c, 2};  rem = cases{c, 3};
    tag = '';  if strcmp(ft, 'open') && strcmp(dn, 'IFOC'), tag = ternary(rem, '_remedy', '_no_remedy'); end
    name = sprintf('%02d_sim_fault_%s_%s%s', 30 + c, dn, ft, tag);
    cap = sprintf('%s, %s fault at 0.8 s (50 %% speed, 50 %% load)%s', dn, ft, strrep(strrep(tag, '_', ' '), 'no remedy', ', no remedy'));
    [f, skip] = newfig(G, name, 16, 15);  if skip, G = note(G, name, cap); continue; end
    if strcmp(dn, 'IFOC'), P = Pi; else, P = Pd; end
    if strcmp(ft, 'open'), P.ctrl.remedy_open = rem; end
    r = sim(G, P, 'fault', struct('fault', ft));
    t = r.t;  ix = t >= 0.7;
    subplot(4, 1, 1);
    wa = r.lp(ix, I.p.w)*30/pi;  wr = r.lc(ix, I.c.w_ref)*30/pi;
    plot(t(ix), r.lc(ix, I.c.w_est)*30/pi, ':', 'Color', [0.6 0.6 0.6]);  hold on;
    plot(t(ix), wa, 'Color', G.col.(dn), 'LineWidth', 1.5);  plot(t(ix), wr, 'k--');
    lo = min([wa; wr]);  hi = max([wa; wr]);  pad = max(0.15*(hi - lo), 20);
    ylim([lo - pad, hi + pad]);
    ylabel('speed [rpm]');  legend({'estimated (MRAS)', 'actual', 'reference'}, 'Location', 'best', 'FontSize', 8);
    grid on;  title(cap, 'FontWeight', 'normal', 'FontSize', 9);  xline(0.8, 'r:', 'HandleVisibility', 'off');
    subplot(4, 1, 2);
    plot(t(ix), r.lp(ix, I.p.Te), t(ix), r.lc(ix, I.c.Te_ref), '--');
    ylabel('T_e [N m]');  legend({'actual', 'reference'}, 'Location', 'best', 'FontSize', 8);  grid on;  xline(0.8, 'r:', 'HandleVisibility', 'off');
    subplot(4, 1, 3);
    plot(t(ix), r.lp(ix, [I.p.ia I.p.ib I.p.ic]));
    ylabel('current [A]');  legend({'i_a', 'i_b', 'i_c'}, 'Location', 'best', 'FontSize', 8);  grid on;  xline(0.8, 'r:', 'HandleVisibility', 'off');
    subplot(4, 1, 4);
    fl = [r.lc(ix, I.c.f_speed), r.lc(ix, I.c.f_cur), r.lc(ix, I.c.f_open), r.lc(ix, I.c.neutral)];
    hold on;
    for q = 1:4, stairs(t(ix), 0.8*(fl(:, q) > 0.5) + 1.2*(q - 1)); end
    set(gca, 'YTick', 0.4 + 1.2*(0:3), 'YTickLabel', {'speed fault', 'current fault', 'open phase', 'neutral remedy'}, 'FontSize', 8);
    ylim([-0.2 4.6]);  xlabel('time [s]');  grid on;  xline(0.8, 'r:', 'HandleVisibility', 'off');
    G = savefig_all(G, f, name, cap);
end
end

function G = fig_sim_flux_strategies(G)
name = '40_sim_flux_strategies_load_step';  cap = 'Load step with rated flux, analytic LUT and ML flux: flux reference, torque, speed';
[f, skip] = newfig(G, name, 16, 13);  if skip, G = note(G, name, cap); return; end
[Pi, Pd] = params(G);  I = log_index();
[Pi_ml, Pd_ml] = ml_params(G, Pi, Pd);
dn = {'IFOC', 'DTC'};  Ps = {Pi, Pi, Pi_ml; Pd, Pd, Pd_ml};  fl = {'rated', 'LUT', 'ML'};
for d = 1:2
    R = cell(1, 3);
    for j = 1:3
        P = Ps{d, j};
        if j == 1, P.ctrl.flux_mode = 1; elseif j == 2, P.ctrl.flux_mode = 3; end
        R{j} = sim(G, P, 'load_step');
    end
    subplot(3, 2, d);  hold on;
    for j = 1:3, plot(R{j}.t, R{j}.lc(:, I.c.psi_ref), 'Color', colof(G, fl{j})); end
    ylabel('\psi_r^* [Wb]');  title(dn{d});  grid on;  if d == 1, legend(fl, 'Location', 'southeast'); end
    subplot(3, 2, 2 + d);  hold on;
    for j = 1:3, plot(R{j}.t, R{j}.lp(:, I.p.Te), 'Color', colof(G, fl{j})); end
    ylabel('T_e [N m]');  grid on;  xlim([0.7 1.0]);
    subplot(3, 2, 4 + d);  hold on;
    for j = 1:3, plot(R{j}.t, R{j}.lp(:, I.p.w)*30/pi, 'Color', colof(G, fl{j})); end
    ylabel('speed [rpm]');  xlabel('time [s]');  grid on;  xlim([0.7 1.0]);
end
G = savefig_all(G, f, name, cap);
end

function G = fig_agv_traces(G)
[Pi, Pd] = params(G);
cases = {'IFOC', 0; 'DTC', 0; 'IFOC', 3; 'DTC', 3};
for c = 1:size(cases, 1)
    dn = cases{c, 1};  ft = cases{c, 2};
    fname = ternary(ft == 0, 'healthy', 'open_phase');
    name = sprintf('%02d_agv_mission_%s_%s', 50 + c, dn, fname);
    cap = sprintf('AGV mission, %s, %s: path, speed, energy (rated vs analytic LUT flux)', dn, strrep(fname, '_', ' '));
    [f, skip] = newfig(G, name, 16, 13);  if skip, G = note(G, name, cap); continue; end
    if strcmp(dn, 'IFOC'), P0 = Pi; else, P0 = Pd; end
    R = cell(1, 2);  fl = [1 3];  ln = {'rated', 'LUT'};
    for j = 1:2
        P = P0;  P.ctrl.flux_mode = fl(j);  P.agv.fault_type = ft;
        fprintf('  AGV %s %s flux %d (a few minutes) ...\n', dn, fname, fl(j));
        R{j} = run_agv(P, G.opt.engine);
    end
    pth = agv_path(P0);
    subplot(2, 2, 1);  plot(pth.wp(:, 1), pth.wp(:, 2), 'k--');  hold on;
    for j = 1:2, plot(R{j}.la(:, 1), R{j}.la(:, 2), 'Color', colof(G, ln{j})); end
    axis equal;  grid on;  xlabel('x [m]');  ylabel('y [m]');  legend({'path', 'rated', 'LUT'}, 'Location', 'best', 'FontSize', 8);
    subplot(2, 2, 2);  hold on;
    for j = 1:2, plot(R{j}.t, R{j}.la(:, 4), 'Color', colof(G, ln{j})); end
    plot(R{1}.t, R{1}.la(:, 6), 'k:');  grid on;  xlabel('time [s]');  ylabel('v [m/s]');
    subplot(2, 2, 3);  hold on;
    for j = 1:2, plot(R{j}.t, R{j}.la(:, 7), 'Color', colof(G, ln{j})); end
    grid on;  xlabel('time [s]');  ylabel('energy [Wh]');
    subplot(2, 2, 4);  hold on;
    for j = 1:2, plot(R{j}.t, 100*R{j}.la(:, 8), 'Color', colof(G, ln{j})); end
    grid on;  xlabel('time [s]');  ylabel('cross-track error [cm]');
    sgtitle(cap, 'FontSize', 9);
    G = savefig_all(G, f, name, cap);
end
end

function out = ternary(c, a, b)
if c, out = a; else, out = b; end
end
