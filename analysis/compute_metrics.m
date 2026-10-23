function M = compute_metrics(res)
%COMPUTE_METRICS  All performance metrics of one run (proposal section 5.6).
%
%   M = compute_metrics(res)    res from run_sim
%
%   Dynamic:   rise_time [s], overshoot [%], settling_time [s] (2% band),
%              itae, load_dip_rpm, load_recovery [s]
%   Ripple:    Te_ripple_pp [% of rated], Te_ripple_rms [% of rated]
%   Current:   thd50 [%] (harmonics 2..50), thd_total [%] (incl. switching ripple)
%   Inverter:  fsw_avg [Hz] (average device switching frequency)
%   Power:     P_in, P_el, P_cond, P_sw, P_fe, P_cus, P_cur, P_out, P_fric [W],
%              eta_drive, eta_motor
%   Faults:    t_detect [ms], post_* metrics in window win.ss2
P = res.P;  scen = res.scen;  win = scen.win;  I = log_index();
t = res.t;  lp = res.lp;  lc = res.lc;
Ts = t(2) - t(1);
w = lp(:, I.p.w);  wref = lc(:, I.c.w_ref);
Tr = P.m.T_rated;

M = struct();
% ---------------- speed step ----------------
M.rise_time = NaN; M.overshoot = NaN; M.settling_time = NaN;
if isfinite(win.t_step)
    t_end = t(end);  if isfinite(win.t_load), t_end = win.t_load; end
    ix = t >= win.t_step & t < t_end;
    tt = t(ix) - win.t_step;  y = w(ix);
    dy = win.step_to - win.step_from;
    yn = (y - win.step_from)/dy;              % normalised 0 -> 1
    k10 = find(yn >= 0.1, 1);  k90 = find(yn >= 0.9, 1);
    if ~isempty(k10) && ~isempty(k90), M.rise_time = tt(k90) - tt(k10); end
    M.overshoot = max(0, (max(yn) - 1)*100);
    out = find(abs(yn - 1) > 0.02, 1, 'last');
    if isempty(out), M.settling_time = 0; elseif out < numel(tt), M.settling_time = tt(out+1); end
end
% ---------------- ITAE ----------------
M.itae = NaN;
if all(isfinite(win.itae))
    ix = t >= win.itae(1) & t <= win.itae(2);
    M.itae = sum((t(ix) - win.itae(1)).*abs(wref(ix) - w(ix)))*Ts;
end
% ---------------- load step ----------------
M.load_dip_rpm = NaN;  M.load_recovery = NaN;
if isfinite(win.t_load)
    ix = t >= win.t_load;
    e = wref(ix) - w(ix);  tt = t(ix) - win.t_load;
    M.load_dip_rpm = max(e)*60/(2*pi);
    tol = 0.01*max(abs(wref(ix)));
    out = find(abs(e) > tol, 1, 'last');
    if isempty(out), M.load_recovery = 0; elseif out < numel(tt), M.load_recovery = tt(out+1); else, M.load_recovery = Inf; end
end
% ---------------- steady-state window ----------------
M = add_window_metrics(M, '', res, win.ss, Tr, Ts);
% ---------------- faults ----------------
M.t_detect = NaN;
if isfinite(win.t_fault)
    f = lc(:, I.c.f_speed) + lc(:, I.c.f_cur) + lc(:, I.c.f_open);
    k = find(t >= win.t_fault & f > 0, 1);
    if ~isempty(k), M.t_detect = (t(k) - win.t_fault)*1e3; end
    M = add_window_metrics(M, 'post_', res, win.ss2, Tr, Ts);
    ix = t >= win.ss2(1) & t <= win.ss2(2);
    M.post_speed_err_pct = mean(abs(wref(ix) - w(ix)))/max(abs(wref(ix)))*100;
    M.post_Te_mean = mean(lp(ix, I.p.Te));
    M.post_I_peak = max(max(abs(lp(ix, [I.p.ia I.p.ib I.p.ic]))));
end
end

% =====================================================================
function M = add_window_metrics(M, pre, res, wn, Tr, Ts)
I = log_index();  lp = res.lp;  t = res.t;
names = {'Te_ripple_pp','Te_ripple_rms','thd50','thd_total','fsw_avg','P_in','P_el','P_cond', ...
         'P_sw','P_fe','P_cus','P_cur','P_out','P_fric','eta_drive','eta_motor','w_mean'};
for i = 1:numel(names), M.([pre names{i}]) = NaN; end
if ~all(isfinite(wn)), return; end
ix = t >= wn(1) & t <= wn(2);
if sum(ix) < 10, return; end
Te = lp(ix, I.p.Te);
M.([pre 'Te_ripple_pp'])  = (max(lp(ix, I.p.Te_max)) - min(lp(ix, I.p.Te_min)))/Tr*100;
M.([pre 'Te_ripple_rms']) = sqrt(max(mean(lp(ix, I.p.Te2)) - mean(Te)^2, 0))/Tr*100;
[M.([pre 'thd50']), M.([pre 'thd_total'])] = current_thd(lp(ix, I.p.ia_avg), lp(ix, I.p.ia2), Ts);
M.([pre 'fsw_avg']) = sum(lp(ix, I.p.nsw))/(3*2*sum(ix)*Ts);
pw = mean(lp(ix, [I.p.Pel I.p.Pcond I.p.Psw I.p.Pfe I.p.Pcus I.p.Pcur I.p.Pout I.p.Pfric]), 1);
M.([pre 'P_el']) = pw(1);  M.([pre 'P_cond']) = pw(2);  M.([pre 'P_sw']) = pw(3);
M.([pre 'P_fe']) = pw(4);  M.([pre 'P_cus']) = pw(5);  M.([pre 'P_cur']) = pw(6);
M.([pre 'P_out']) = pw(7); M.([pre 'P_fric']) = pw(8);
M.([pre 'P_in']) = pw(1) + pw(2) + pw(3) + pw(4);
M.([pre 'eta_drive']) = pw(7)/M.([pre 'P_in']);
M.([pre 'eta_motor']) = pw(7)/(pw(1) + pw(4));
M.([pre 'w_mean']) = mean(lp(ix, I.p.w));
end

function [thd50, thd_tot] = current_thd(ia_avg, ia2, Ts)
% THD of phase-a current. Samples are step averages, resampled to 100 us
% blocks so IFOC (100 us) and DTC (25 us) are treated identically.
nb = max(1, round(100e-6/Ts));
n = floor(numel(ia_avg)/nb)*nb;
x = mean(reshape(ia_avg(1:n), nb, []), 1)';
y2 = mean(reshape(ia2(1:n), nb, []), 1)';
Tb = nb*Ts;
% fundamental from zero crossings; keep an integer number of cycles
% zero crossings of a smoothed copy (1 ms moving average), spurious ones removed
nf = max(1, round(1e-3/Tb));
xs = filter(ones(nf,1)/nf, 1, x);
zc = find(xs(1:end-1) < 0 & xs(2:end) >= 0);
if numel(zc) >= 3
    g = diff(zc);
    keep = [true; g >= 0.6*median(g)];
    zc = zc(keep);
end
thd50 = NaN;  thd_tot = NaN;
if numel(zc) < 3, return; end
x = x(zc(1):zc(end)-1);
y2 = y2(zc(1):zc(end)-1);
ncyc = numel(zc) - 1;
N = numel(x);  fe = ncyc/(N*Tb);
tt = (0:N-1)'*Tb;
A = zeros(50,1);
for h = 1:50
    A(h) = abs(2/N*sum(x.*exp(-1j*2*pi*h*fe*tt)));
end
thd50 = sqrt(sum(A(2:50).^2))/A(1)*100;
Irms2 = mean(y2);
I1 = A(1)/sqrt(2);
thd_tot = sqrt(max(Irms2 - I1^2, 0))/I1*100;
end
