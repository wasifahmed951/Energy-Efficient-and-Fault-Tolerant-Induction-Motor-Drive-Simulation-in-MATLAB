function T = run_flux_sweep(P, opts)
%RUN_FLUX_SWEEP  Steady-state speed x torque x flux sweep (proposal 5.4, step 1).
%
%   T = run_flux_sweep(P)            proposal grid: 10 speeds x 8 torques x 13 fluxes
%   T = run_flux_sweep(P, opts)      opts.w, opts.T, opts.psi (vectors), opts.engine,
%                                    opts.parallel, opts.out (CSV file name)
%
%   Each run holds the flux at a fixed value (flux_mode = 2), runs the
%   'steady' cycle and averages the last 0.25 s. Writes:
%     results/flux_sweep_<IFOC|DTC>.csv   one row per run (training data for Python)
%     results/flux_opt_<IFOC|DTC>.csv     loss-minimising flux per (w, T)
wr = P.m.wm_rated;  Tr = P.m.T_rated;
if nargin < 2, opts = struct(); end
wv = getf(opts, 'w', linspace(0.1, 1.0, 10)*wr);
Tv = getf(opts, 'T', linspace(0.1, 0.8, 8)*Tr);
pv = getf(opts, 'psi', linspace(0.4, 1.0, 13)*P.m.psi_r_rated);
engine = getf(opts, 'engine', 'simulink');
par = getf(opts, 'parallel', true);
tag = 'IFOC';  if P.ctrl.type == 2, tag = 'DTC'; end
root = fileparts(fileparts(mfilename('fullpath')));
out = getf(opts, 'out', fullfile(root, 'results', ['flux_sweep_' tag '.csv']));

[W, TT, PS] = ndgrid(wv, Tv, pv);
n = numel(W);
Pl = cell(n,1);
for i = 1:n
    Pi = P;  Pi.ctrl.flux_mode = 2;  Pi.ctrl.psi_fixed = PS(i);
    Pi.ctrl.trans_dw = 1e9;  Pi.ctrl.trans_dT = 1e9;     % no rated-flux fallback
    Pl{i} = Pi;
end
Sl = @(i) make_scenario('steady', Pl{i}, struct('w', W(i), 'T', TT(i), 'Tend', 1.0));
fprintf('Flux sweep (%s): %d runs\n', tag, n);
cols = {'w','T','psi','P_in','P_el','P_cond','P_sw','P_fe','P_cus','P_cur','P_out','P_fric', ...
        'eta_drive','eta_motor','Te_ripple_rms','thd50','speed_err_pct','ok'};
I = log_index();
red = @(r, i) sweep_row(r, i, W, TT, PS, I);
R = run_batch(Pl, Sl, engine, par, struct('reduce', red, 'chunk', 48, ...
    'ckpt', fullfile(root, 'results', ['ckpt_flux_sweep_' tag '.mat'])));
D = NaN(n, numel(cols));
for i = 1:n
    if isempty(R{i}), D(i,1:3) = [W(i) TT(i) PS(i)];  D(i,end) = 0; else, D(i,:) = R{i}; end
end
write_csv(out, cols, D);
fprintf('Wrote %s\n', out);
% loss-minimising flux per operating point
G = unique(D(:,1:2), 'rows');
O = NaN(size(G,1), 6);
for g = 1:size(G,1)
    sel = D(:,1) == G(g,1) & D(:,2) == G(g,2) & D(:,end) == 1;
    if ~any(sel), continue; end
    Ds = D(sel,:);
    [~, k] = min(Ds(:,4));
    rated = Ds(abs(Ds(:,3) - P.m.psi_r_rated) < 1e-9, 4);
    if isempty(rated), rated = NaN; end
    O(g,:) = [G(g,:) Ds(k,3) Ds(k,4) rated(1) (rated(1) - Ds(k,4))/rated(1)*100];
end
O = O(all(isfinite(O(:,1:4)), 2), :);
[od, on, oe] = fileparts(out);
out2 = fullfile(od, ['flux_opt_' strrep(on, 'flux_sweep_', '') oe]);
write_csv(out2, {'w','T','psi_opt','P_in_opt','P_in_rated','saving_pct'}, O);
fprintf('Wrote %s\n', out2);
T = struct('runs', D, 'cols', {cols}, 'opt', O);
end

function row = sweep_row(r, i, W, TT, PS, I)
M = compute_metrics(r);
ix = r.t >= r.scen.win.ss(1);
werr = mean(abs(r.lc(ix, I.c.w_ref) - r.lp(ix, I.p.w)))/max(W(i), 1)*100;
ok = werr < 1 && isfinite(M.P_in);
row = [W(i) TT(i) PS(i) M.P_in M.P_el M.P_cond M.P_sw M.P_fe M.P_cus M.P_cur M.P_out ...
       M.P_fric M.eta_drive M.eta_motor M.Te_ripple_rms M.thd50 werr ok];
end

function v = getf(s, f, d)
if isfield(s, f), v = s.(f); else, v = d; end
end
