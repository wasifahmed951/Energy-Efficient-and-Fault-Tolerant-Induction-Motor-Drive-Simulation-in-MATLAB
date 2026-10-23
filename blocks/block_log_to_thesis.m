function [t, lp, lc] = block_log_to_thesis(tt, X, P)
%BLOCK_LOG_TO_THESIS  Turn the block model's log (bm_log, every solver step) into the
%   per-control-step logs of the thesis engine, so compute_metrics.m can be used unchanged.
%
%   [t, lp, lc] = block_log_to_thesis(bm_log.time, bm_log.signals.values, P)
%
%   Row k covers control step k (from (k-1)*Ts to k*Ts) and is stamped t = k*Ts,
%   exactly like run_sim. Averages of powers, torque and current come from the
%   energy-meter integrals (exact), and so do the mean squares of the torque and the
%   phase-a current (rms ripple, total THD; plant_step.m integrates them exactly too).
%   The torque min/max are taken at the thesis evaluation points (plant_step.m): the
%   PWM switching instants and sub-steps at most P.sim.h_max apart.
%
%   bm_log columns: 1 w_ref 2 w 3 Te 4 ia 5 ib 6 ic 7 Pel 8 Pcond 9 Psw 10 Pfe 11 Pout
%   12 Pfric 13 transitions 14 f_speed 15 f_cur 16 f_open 17 neutral 18 w_est 19 Te_ref
%   20 psi_ref 21 psi_est 22 |psi_r| 23 |psi_m| 24 mode 25 E_el 26 E_cond 27 E_sw 28 E_fe
%   29 E_out 30 E_fric 31 int_Te 32 int_ia 33 da 34 db 35 dc 36 int_Te2 37 int_ia2

tt = tt(:);
X = squeeze(X);
if size(X, 1) ~= numel(tt), X = X'; end
Ts = P.ctrl.Ts;  h = tt(2) - tt(1);
NT = round(Ts/h);  hmax = P.sim.h_max;
K = floor((numel(tt) - 1)/NT);
lp = zeros(K, 23);  lc = zeros(K, 16);
t = (1:K)'*Ts;
n0 = (0:K-1)'*NT + 1;  n1 = n0 + NT;
% end-of-step values and exact step averages
lp(:, 1:3) = X(n1, 4:6);
lp(:, 4) = X(n1, 2);
lp(:, 5) = (X(n1, 31) - X(n0, 31))/Ts;
lp(:, 9) = (X(n1, 25) - X(n0, 25))/Ts;
lp(:, 10) = (X(n1, 26) - X(n0, 26))/Ts;
lp(:, 11) = (X(n1, 27) - X(n0, 27))/Ts;
lp(:, 12) = (X(n1, 28) - X(n0, 28))/Ts;
lp(:, 13:14) = NaN;                       % copper losses are not logged by the block model
lp(:, 15) = X(n1, 22);
lp(:, 16) = X(n1, 23);
lp(:, 17) = (X(n1, 32) - X(n0, 32))/Ts;
lp(:, 8) = (X(n1, 36) - X(n0, 36))/Ts;
lp(:, 18) = (X(n1, 37) - X(n0, 37))/Ts;
lp(:, 20) = X(n0, 24);
lp(:, 22) = (X(n1, 29) - X(n0, 29))/Ts;
lp(:, 23) = (X(n1, 30) - X(n0, 30))/Ts;
% thesis evaluation points: segment boundaries at the switching instants, sub-steps <= h_max
for k = 1:K
    a = n0(k);
    Te = X(a:a+NT, 3);  ia = X(a:a+NT, 4);
    if X(a+NT, 24) ~= X(a+NT-1, 24) && NT >= 2
        % mode changed exactly at the end of the step: use the left limit
        Te(end) = 2*Te(end-1) - Te(end-2);  ia(end) = 2*ia(end-1) - ia(end-2);
    end
    d = X(a, 33:35);
    if mod(k - 1, 2) == 0, tsw = (1 - d)*Ts; else, tsw = d*Ts; end
    bp = sort([0, tsw, Ts]);
    pts = 0;
    for s = 1:4
        dt = bp(s+1) - bp(s);
        if dt <= 1e-12, continue; end
        ns = max(1, ceil(dt/hmax - 1e-9));
        pts = [pts, bp(s) + (1:ns)*dt/ns]; %#ok<AGROW>
    end
    pts = min(max(pts, 0), Ts);
    x = pts/h;  i0 = min(floor(x + 1e-9), NT - 1);  f = max(x - i0, 0);
    Tp = (1 - f).*Te(i0 + 1)' + f.*Te(i0 + 2)';
    % a switching instant inside a solver step is a corner of the torque waveform, which
    % the solver grid cuts off: rebuild it from the straight lines on both sides
    % (only where no other switching instant is within one solver step)
    for q = find(f > 1e-6 & f < 1 - 1e-6)
        if ~any(abs(tsw - pts(q)) < 1e-12), continue; end
        g = a + i0(q);                                % sample before the corner
        if g < 2 || g + 2 > size(X, 1), continue; end
        others = tsw(abs(tsw - pts(q)) > 1e-12);
        if any(abs(others - pts(q)) < 2*h), continue; end
        s1 = X(g, 3) - X(g-1, 3);  s2 = X(g+2, 3) - X(g+1, 3);
        Tp(q) = 0.5*((X(g, 3) + f(q)*s1) + (X(g+1, 3) - (1 - f(q))*s2));
    end
    lp(k, 6) = min(Tp);  lp(k, 7) = max(Tp);
    lp(k, 19) = sum(X(a:a+NT-1, 13));
end
% controller log: the controller of step k runs at the start of the step
lc(:, 2) = X(n0, 18);   lc(:, 3) = X(n0, 19);  lc(:, 4) = X(n0, 20);  lc(:, 5) = X(n0, 21);
lc(:, 11) = X(n0, 14);  lc(:, 12) = X(n0, 15); lc(:, 13) = X(n0, 16); lc(:, 14) = X(n0, 17);
lc(:, 16) = X(n0, 1);
lc(:, 1) = X(n0, 2);
end
