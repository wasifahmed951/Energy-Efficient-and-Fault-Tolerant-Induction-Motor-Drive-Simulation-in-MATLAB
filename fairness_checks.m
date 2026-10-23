%% fairness_checks.m  -  two short checks before the results are reported (about 20-30 min)
%
%  CHECK 1 - tuning bounds.  The GA/PSO gains of section 4 ended exactly on the upper
%            bounds of tuning_bounds() (IFOC: Kp_w = 10, Ki_w = 1000, Ki_i = 10^4.7;
%            DTC: Kp_w = 10, Ki_w = 1000). One PSO run per drive is repeated with the
%            upper bounds one decade (10x) higher, same budget (600 evaluations), seed 1.
%            -> shows how much J would still improve and how large the gains would become.
%
%  CHECK 2 - switching-frequency fairness.  The DTC bands were calibrated to 5 kHz at
%            one steady point (50 % speed, 50 % load), but on the test cycles DTC switched
%            at 2.6-5.7 kHz. Here the bands are re-calibrated PER TEST CYCLE so that DTC's
%            average switching frequency equals 5 kHz on that cycle, and the cycle is run
%            again -> torque ripple / THD of DTC vs IFOC at truly matched frequency.
%
%  Uses results/tuned_params.mat (Pi_zn, Pd_zn, Pi_best, Pd_best) and tuning_runs.mat.
%  Writes results/fairness_checks.mat and results/table_fsw_matched.csv.
%  Run from the "Electrical Drive" folder:   fairness_checks

function fairness_checks()
clc;
setup_project();
ENGINE = 'simulink';
root = fileparts(mfilename('fullpath'));
rd = fullfile(root, 'results');
diary(fullfile(rd, sprintf('log_fairness_%s.txt', datestr(now, 'yyyymmdd_HHMM'))));
S = load(fullfile(rd, 'tuned_params.mat'), 'Pi_zn', 'Pd_zn', 'Pi_best', 'Pd_best');
U = load(fullfile(rd, 'tuning_runs.mat'), 'tun');
out = struct();

%% ================= CHECK 1: wider tuning bounds =================
fprintf('\n================ CHECK 1: tuning with 10x wider upper bounds ================\n');
if has_toolbox('pct')
    pool = gcp();  wait(parfevalOnAll(pool, @worker_setup, 0));
end
base = {'IFOC', S.Pi_zn; 'DTC', S.Pd_zn};
for d = 1:2
    tag = base{d,1};  P0 = base{d,2};
    [lb, ub, names] = tuning_bounds(P0);
    ub2 = ub + 1;                                        % log10 units: one decade higher
    fun = @(x) tuning_cost(x(:)', P0, ENGINE);
    rng(1);
    t0 = tic;
    if has_toolbox('gads')
        o = optimoptions('particleswarm', 'SwarmSize', 20, 'MaxIterations', 30, ...
            'MaxStallIterations', 1e6, 'FunctionTolerance', 1e-12, 'Display', 'iter', ...
            'UseParallel', has_toolbox('pct'));
        [xb, Jb] = particleswarm(fun, numel(lb), lb, ub2, o);
    else
        [xb, Jb] = pso_basic(fun, lb, ub2, 20, 600, 1, []);
    end
    % best J of the original section-4 runs (same drive, any optimiser)
    Jold = Inf;  xold = [];
    for k = 1:numel(U.tun)
        if strcmp(U.tun(k).drive, tag) && U.tun(k).out.J < Jold
            Jold = U.tun(k).out.J;  xold = U.tun(k).out.x;
        end
    end
    at_new_bound = abs(xb - ub2) < 1e-3;
    % speed-loop bandwidth estimate: w_bw ~ Kp_w / J_motor
    bw_old = 10^xold(1)/P0.m.J/(2*pi);  bw_new = 10^xb(1)/P0.m.J/(2*pi);
    fprintf('\n%s  variables: %s\n', tag, strjoin(cellstr(names), ', '));
    fprintf('   original bounds (log10): lb %s  ub %s\n', mat2str(lb, 3), mat2str(ub, 3));
    fprintf('   wider    bounds (log10): lb %s  ub %s\n', mat2str(lb, 3), mat2str(ub2, 3));
    fprintf('   original best : J = %.5f  gains = %s\n', Jold, mat2str(10.^xold, 4));
    fprintf('   wider bounds  : J = %.5f  gains = %s   (%.0f s)\n', Jb, mat2str(10.^xb, 4), toc(t0));
    fprintf('   J improvement : %.2f %%\n', 100*(Jold - Jb)/Jold);
    fprintf('   still on the (new) upper bound: %s\n', mat2str(at_new_bound));
    fprintf('   speed-loop bandwidth ~Kp_w/J: %.0f Hz (original) -> %.0f Hz (wider);  current loop ~500 Hz\n', bw_old, bw_new);
    out.check1.(tag) = struct('lb', lb, 'ub', ub, 'ub_wide', ub2, 'names', {names}, 'x_old', xold, 'J_old', Jold, ...
        'x_new', xb, 'J_new', Jb, 'improvement_pct', 100*(Jold - Jb)/Jold, 'at_new_bound', at_new_bound, ...
        'bw_old_Hz', bw_old, 'bw_new_Hz', bw_new);
end
save(fullfile(rd, 'fairness_checks.mat'), 'out');

%% ================= CHECK 2: DTC at matched switching frequency per cycle =================
fprintf('\n================ CHECK 2: DTC bands re-calibrated to 5 kHz on each test cycle ================\n');
Pi = S.Pi_best;  Pi.ctrl.flux_mode = 1;
Pd = S.Pd_best;  Pd.ctrl.flux_mode = 1;
f_target = Pi.inv.fsw;
hT0 = Pd.ctrl.h_T;  hp0 = Pd.ctrl.h_psi;
cycles = {'step_rated', 'load_step', 'reversal'};
rows = {};
for c = 1:numel(cycles)
    cy = cycles{c};
    % IFOC reference
    Mi = compute_metrics(run_sim(Pi, make_scenario(cy, Pi), ENGINE));
    % DTC with the section-2 bands (as used in all results)
    Md0 = compute_metrics(run_sim(Pd, make_scenario(cy, Pd), ENGINE));
    % DTC with bands re-calibrated on this cycle (bisection on a common scale factor)
    lo = 0.1;  hi = 3;  best = [Inf, 1];
    for it = 1:8
        k = sqrt(lo*hi);
        Pk = Pd;  Pk.ctrl.h_T = k*hT0;  Pk.ctrl.h_psi = k*hp0;
        Mk = compute_metrics(run_sim(Pk, make_scenario(cy, Pk), ENGINE));
        fprintf('   %s: band scale %.3f -> fsw %.0f Hz\n', cy, k, Mk.fsw_avg);
        if abs(Mk.fsw_avg - f_target) < best(1), best = [abs(Mk.fsw_avg - f_target), k]; end
        if Mk.fsw_avg > f_target, lo = k; else, hi = k; end
    end
    Pk = Pd;  Pk.ctrl.h_T = best(2)*hT0;  Pk.ctrl.h_psi = best(2)*hp0;
    Md = compute_metrics(run_sim(Pk, make_scenario(cy, Pk), ENGINE));
    rows(end+1, :) = metrics_row(cy, 'IFOC', 1, Mi); %#ok<AGROW>
    rows(end+1, :) = metrics_row(cy, 'DTC (section-2 bands)', 1, Md0); %#ok<AGROW>
    rows(end+1, :) = metrics_row(cy, 'DTC (matched 5 kHz)', best(2), Md); %#ok<AGROW>
end
T = cell2table(rows, 'VariableNames', {'cycle', 'drive', 'band_scale', 'fsw_Hz', 'Te_ripple_pp', ...
    'Te_ripple_rms', 'thd50', 'thd_total', 'overshoot', 'itae', 'load_dip_rpm', 'P_sw', 'eta_drive'});
writetable(T, fullfile(rd, 'table_fsw_matched.csv'));
out.check2 = T;
save(fullfile(rd, 'fairness_checks.mat'), 'out');
disp(T);
fprintf('\nDone. CHECK 1 and CHECK 2 results are printed above\n');
fprintf('(also saved in results\\table_fsw_matched.csv and the log file results\\log_fairness_*.txt).\n');
diary off
end

function r = metrics_row(cy, name, k, M)
g = @(f) getm(M, f);
r = {cy, name, k, g('fsw_avg'), g('Te_ripple_pp'), g('Te_ripple_rms'), g('thd50'), g('thd_total'), ...
     g('overshoot'), g('itae'), g('load_dip_rpm'), g('P_sw'), g('eta_drive')};
end

function v = getm(M, f)
if isfield(M, f) && ~isempty(M.(f)), v = double(M.(f)); else, v = NaN; end
end
