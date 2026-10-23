function [P, info] = calibrate_dtc_bands(P, f_target, engine)
%CALIBRATE_DTC_BANDS  Scale the DTC hysteresis bands so the average device
%   switching frequency equals f_target (fairness rule: matches the IFOC
%   carrier, default P.inv.fsw = 5 kHz). Bisection on a common scale factor
%   at 0.5 rated speed and 50% load.
if nargin < 2 || isempty(f_target), f_target = P.inv.fsw; end
if nargin < 3, engine = 'simulink'; end
hT0 = P.ctrl.h_T;  hp0 = P.ctrl.h_psi;
scen = make_scenario('steady', P, struct('w', 0.5*P.m.wm_rated, 'T', 0.5*P.m.T_rated, 'Tend', 0.8));
lo = 0.1;  hi = 3;   % band scale (smaller band -> higher frequency)
best = [Inf, 1, NaN];   % [|error|, scale, fsw]
for it = 1:8
    k = sqrt(lo*hi);
    P.ctrl.h_T = k*hT0;  P.ctrl.h_psi = k*hp0;
    M = compute_metrics(run_sim(P, scen, engine));
    fprintf('  band scale %.3f -> fsw %.0f Hz\n', k, M.fsw_avg);
    if abs(M.fsw_avg - f_target) < best(1), best = [abs(M.fsw_avg - f_target), k, M.fsw_avg]; end
    if M.fsw_avg > f_target, lo = k; else, hi = k; end
end
k = best(2);
P.ctrl.h_T = k*hT0;  P.ctrl.h_psi = k*hp0;
info = struct('scale', k, 'h_T', P.ctrl.h_T, 'h_psi', P.ctrl.h_psi, 'fsw', best(3));
fprintf('DTC bands: h_T = %.3f N m, h_psi = %.4f Wb -> %.0f Hz (target %.0f Hz)\n', ...
    P.ctrl.h_T, P.ctrl.h_psi, info.fsw, f_target);
end
