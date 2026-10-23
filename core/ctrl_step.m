function [u, logc, st] = ctrl_step(st, meas, ref, P)
%CTRL_STEP  One sample of the drive controller (IFOC or DTC, set by P.ctrl.type).
%
%   meas = [ia_m; ib_m; ic_m; w_m]   measured phase currents [A] and speed [rad/s]
%   ref  = [w_ref; ...]              only ref(1) (speed reference) is used here
%   u    = [d_a; d_b; d_c; neutral]  leg duties (0..1) and neutral contactor command
%   logc = 16 logged quantities, see sim/log_index.m (I.c.*)
%
%   Shared by both drives: speed PI (or relay for the ZN test), flux
%   reference (rated / fixed / LUT / MLP with transient fallback), current
%   observer, MRAS speed estimator, sensor and open-phase fault detection
%   with reconfiguration.

c = P.ctrl;  m = P.m;  Vdc = P.inv.Vdc;  Ts = c.Ts;
st.k = st.k + 1;
t = st.k*Ts;
w_ref = ref(1);
ia_m = meas(1);  ib_m = meas(2);  ic_m = meas(3);  w_m = meas(4);

% ---------------- controller model inductances ----------------
if c.ctrl_sat_aware > 0
    Lm_c = sat_Lms(max(st.psi_ref, 0.05), m);
else
    Lm_c = m.Lm_rated;
end
Ls_c = m.Lls + Lm_c;  Lr_c = m.Llr + Lm_c;
sLs_c = Ls_c - Lm_c^2/Lr_c;  tr_c = Lr_c/m.Rr;

% ---------------- current reconstruction ----------------
obs_is = obs_currents(st.obs, Ls_c, Lr_c, Lm_c);
st.ia_hat = obs_is(1);
if st.neutral > 0.5
    % neutral connected after an open phase: ia = 0, use sensors b and c
    i_ab = [-(ib_m + ic_m)/3; (ib_m - ic_m)/sqrt(3)];
else
    if st.f_cur > 0.5, ia_u = st.ia_hat; else, ia_u = ia_m; end
    ic_u = -ia_u - ib_m;                       % two-sensor scheme, isolated neutral
    i_ab = [ia_u; (ib_m - ic_u)/sqrt(3)];
end

% ---------------- MRAS speed estimator ----------------
i_avg = 0.5*(i_ab + st.i_prev);
st.mr_psv = st.mr_psv + Ts*(st.v_app - m.Rs*i_avg - c.mras_wc*st.mr_psv);
psrv = (Lr_c/Lm_c)*(st.mr_psv - sLs_c*i_ab);
wr_hat = m.p*st.w_est;
pri = st.mr_pri;
st.mr_pri = pri + Ts*((Lm_c/tr_c)*i_ab - pri/tr_c + wr_hat*[-pri(2); pri(1)]);
st.mr_lpf = st.mr_lpf + Ts*c.mras_wc*(st.mr_pri - st.mr_lpf);
pri_hp = st.mr_pri - st.mr_lpf;               % same high-pass as the voltage model
eps_m = (psrv(2)*pri_hp(1) - psrv(1)*pri_hp(2)) / (norm(psrv)*norm(pri_hp) + 1e-3);
st.mr_int = st.mr_int + Ts*c.mras_Ki*eps_m;
st.w_est = (c.mras_Kp*eps_m + st.mr_int)/m.p;
% MRAS validity (16 Oct 2026, from diag_agv2). The MRAS is not observable
% near standstill: at an AGV station its estimate swung to -480..+220 rad/s,
% and after restarting it was still wrong when the speed check switched on.
% That gave false speed-sensor alarms, and after a real encoder fault the
% drive was steered by that estimate and could not leave the station.
%  - below 10% speed the estimate is re-synchronised to the speed in use
%    (encoder, or the reference once the encoder has failed), so the drive
%    runs open-loop in speed there ("I-f" operation) and the MRAS restarts
%    from a sensible value;
%  - the residual is trusted only after 0.5 s in the valid region and while
%    the torque command is not at its limit (at minimum flux and full torque
%    the MRAS error reached 28 rad/s with a healthy encoder).
wmin = 0.1*m.wm_rated;
if abs(w_ref) < wmin
    st.mras_t = 0;
    if st.f_speed > 0.5, w_sync = w_ref; else, w_sync = w_m; end
    st.mr_int = m.p*w_sync;  st.w_est = w_sync;
elseif st.trans_t > 0 || abs(st.dpsi) > 0.01
    st.mras_t = 0;          % flux changing: the MRAS model lags the real flux
else
    st.mras_t = st.mras_t + Ts;
end
mras_ok = st.mras_t >= 0.5 && abs(st.Te_ref) < 0.98*st.Te_lim;

% ---------------- fault detection ----------------
if c.fd_on > 0.5 && t > 0.05
    % speed sensor
    d3w = w_m - 3*st.hw(1) + 3*st.hw(2) - st.hw(3);
    % The MRAS residual is only trusted while the phase-a current is healthy,
    % because a wrong current also corrupts the MRAS estimate.
    % MRAS is not observable near zero frequency, so its residual is only
    % used above 10% of rated speed (an encoder failing at standstill reads
    % ~0 anyway and does no harm there).
    cur_ok = st.f_cur < 0.5 && st.f_open < 0.5 && st.anom < 0.5;
    w_ok = cur_ok && abs(w_ref) > wmin;
    % residual-only path: needs a settled MRAS (the third-difference path
    % catches the abrupt encoder failure and is not gated)
    if w_ok && mras_ok && abs(w_m - st.w_est) > c.res_w_thr, st.cnt_w = st.cnt_w + 1; else, st.cnt_w = 0; end
    if st.f_speed < 0.5 && w_ok && (abs(d3w) > c.tdo_w_thr || st.cnt_w >= c.res_N) && abs(w_m - st.w_est) > c.res_w_thr
        st.f_speed = 1;
        st.Te_lim = min(st.Te_lim, c.Te_max*c.derate_sensor);
    end
    % phase a: sensor fault or open phase. Not while the speed itself is
    % unknown (encoder failed and MRAS not valid): the current observer then
    % runs on the reference speed, its residual exceeds the threshold and a
    % healthy current sensor was flagged (diag_agv2, IFOC speed-fault mission).
    spd_known = st.f_speed < 0.5 || mras_ok;
    if st.f_cur < 0.5 && st.f_open < 0.5 && spd_known
        d3i = ia_m - 3*st.hi(1) + 3*st.hi(2) - st.hi(3);
        res_i = abs(ia_m - st.ia_hat);
        if res_i > c.res_i_thr, st.cnt_i = st.cnt_i + 1; else, st.cnt_i = 0; end
        if st.anom < 0.5 && (abs(d3i) > c.tdo_i_thr || st.cnt_i >= c.res_N)
            st.anom = 1;  st.anom_t = 0;  st.sum_kcl = 0;  st.sum_exp = 0;
        end
        if st.anom > 0.5
            st.anom_t = st.anom_t + Ts;
            st.sum_kcl = st.sum_kcl + abs(ib_m + ic_m);   % = |ia_true| by KCL
            st.sum_exp = st.sum_exp + res_i;
            if st.anom_t >= c.open_N_time
                n = round(st.anom_t/Ts);
                if st.sum_exp/n < c.res_i_thr
                    st.anom = 0;                           % false alarm
                elseif st.sum_kcl < 0.2*st.sum_exp
                    st.f_open = 1;  st.anom = 0;
                    st.Te_lim = min(st.Te_lim, c.Te_max*c.derate_open);
                    if c.type == 1 && c.remedy_open > 0.5, st.neutral = 1; end
                else
                    st.f_cur = 1;  st.anom = 0;
                    st.Te_lim = min(st.Te_lim, c.Te_max*c.derate_sensor);
                end
            end
        end
    end
end
st.hw = [w_m; st.hw(1); st.hw(2)];
st.hi = [ia_m; st.hi(1); st.hi(2)];
if st.f_speed > 0.5, w_u = st.w_est; else, w_u = w_m; end

% ---------------- speed loop ----------------
e_w = w_ref - w_u;
if c.speed_mode == 2
    if e_w > c.relay_h, st.relay = 1; elseif e_w < -c.relay_h, st.relay = -1; end
    Te_ref = c.relay_bias + st.relay*c.relay_T;
else
    Te_u = c.Kp_w*e_w + st.int_w;
    Te_ref = min(max(Te_u, -st.Te_lim), st.Te_lim);
    if Te_ref == Te_u || sign(e_w) ~= sign(Te_u)      % conditional integration
        st.int_w = st.int_w + Ts*c.Ki_w*e_w;
        st.int_w = min(max(st.int_w, -st.Te_lim), st.Te_lim);
    end
end
st.Te_ref = Te_ref;

% ---------------- flux reference ----------------
if abs(e_w) > c.trans_dw || abs(Te_ref - st.Te_ref_f) > c.trans_dT
    st.trans_t = c.trans_hold;
end
st.Te_ref_f = st.Te_ref_f + Ts/0.05*(Te_ref - st.Te_ref_f);
st.trans_t = max(st.trans_t - Ts, 0);
psi_tgt = flux_target(c, m, P.lut, P.mlp, abs(w_u), abs(st.Te_ref_f));
if st.trans_t > 0, psi_tgt = m.psi_r_rated; end
dpsi = min(max(psi_tgt - st.psi_ref, -c.psi_rate*Ts), c.psi_rate*Ts);
st.psi_ref = st.psi_ref + dpsi;
st.dpsi = dpsi/Ts;

% ---------------- inner control ----------------
if c.type == 1
    [d, st] = ifoc_inner(st, i_ab, w_u, Lm_c, Lr_c, sLs_c, tr_c, P);
else
    [d, st] = dtc_inner(st, i_ab, w_u, Lm_c, Lr_c, Ls_c, sLs_c, tr_c, P);
end

% applied voltages for the estimators
vp = (d - 0.5)*Vdc;
st.vp_app = vp;
st.v_app = [(2/3)*(vp(1) - 0.5*(vp(2) + vp(3))); (vp(2) - vp(3))/sqrt(3)];
st.i_prev = i_ab;

% ---------------- propagate the current observer to the next sample ----------------
st.obs = obs_propagate(st.obs, st.v_app, ib_m, m.p*w_u, Ls_c, Lr_c, Lm_c, m, c.Ko, Ts);

u = [d; st.neutral];
logc = zeros(16,1);
logc(1) = w_u;  logc(2) = st.w_est;  logc(3) = Te_ref;
if c.type == 1
    logc(4) = st.psi_ref;  logc(5) = st.psir_est;
    logc(6) = 1.5*m.p*(Lm_c/Lr_c)*st.psir_est*st.iq;
else
    logc(4) = st.psis_ref; logc(5) = norm(st.psis);
    logc(6) = st.Te_est;
end
logc(7) = st.id_ref;  logc(8) = st.iq_ref;  logc(9) = st.id;  logc(10) = st.iq;
logc(11) = st.f_speed;  logc(12) = st.f_cur;  logc(13) = st.f_open;  logc(14) = st.neutral;
logc(15) = st.ia_hat;  logc(16) = w_ref;
end

% =====================================================================
function [d, st] = ifoc_inner(st, i_ab, w_u, Lm_c, Lr_c, sLs_c, tr_c, P)
c = P.ctrl;  m = P.m;  Vdc = P.inv.Vdc;  Ts = c.Ts;
th = st.theta_e;
co = cos(th);  si = sin(th);
id =  co*i_ab(1) + si*i_ab(2);
iq = -si*i_ab(1) + co*i_ab(2);
st.id = id;  st.iq = iq;
% rotor flux (current model) and references
st.psir_est = st.psir_est + Ts*(Lm_c*id - st.psir_est)/tr_c;
psr = max(st.psir_est, 0.05*m.psi_r_rated);
id_ref = st.psi_ref/Lm_c + tr_c*st.dpsi/Lm_c;
id_ref = min(max(id_ref, 0), 0.95*m.I_max);
iq_max = sqrt(max(m.I_max^2 - id_ref^2, 0));
iq_ref = st.Te_ref/(1.5*m.p*(Lm_c/Lr_c)*psr);
iq_ref = min(max(iq_ref, -iq_max), iq_max);
st.id_ref = id_ref;  st.iq_ref = iq_ref;
w_sl = Lm_c*iq_ref/(tr_c*psr);
we = m.p*w_u + w_sl;
% PI current control with decoupling
ed = id_ref - id;  eq = iq_ref - iq;
vd_u = c.Kp_i*ed + st.int_d - we*sLs_c*iq;
vq_u = c.Kp_i*eq + st.int_q + we*(sLs_c*id + (Lm_c/Lr_c)*st.psir_est);
if st.neutral > 0.5, Vmax = Vdc/2; else, Vmax = Vdc/sqrt(3); end
vmag = sqrt(vd_u^2 + vq_u^2);
if vmag > Vmax
    vd = vd_u*Vmax/vmag;  vq = vq_u*Vmax/vmag;     % saturated: hold integrators
else
    vd = vd_u;  vq = vq_u;
    st.int_d = st.int_d + Ts*c.Ki_i*ed;
    st.int_q = st.int_q + Ts*c.Ki_i*eq;
end
% inverse Park at the middle of the next period
thv = th + 0.5*Ts*we;
va = cos(thv)*vd - sin(thv)*vq;
vb = sin(thv)*vd + cos(thv)*vq;
st.theta_e = mod(th + Ts*we, 2*pi);
if st.neutral > 0.5
    % open phase a, neutral connected: remap (u_alpha, u_beta) to legs b and c
    vp = [0; (-va + sqrt(3)*vb)/2; (-va - sqrt(3)*vb)/2];
else
    vabc = [va; -0.5*va + (sqrt(3)/2)*vb; -0.5*va - (sqrt(3)/2)*vb];
    voff = -0.5*(max(vabc) + min(vabc));       % min-max injection = SVPWM
    vp = vabc + voff;
end
d = min(max(0.5 + vp/Vdc, 0), 1);
end

% =====================================================================
function [d, st] = dtc_inner(st, i_ab, w_u, Lm_c, Lr_c, Ls_c, sLs_c, tr_c, P)
c = P.ctrl;  m = P.m;  Ts = c.Ts;
% Stator flux: combined voltage/current-model observer (Jansen-Lorenz type).
%   Current model (alpha-beta, valid down to standstill):
%     d(psi_r)/dt = (Lm/tr)*i_s - psi_r/tr + wr*J*psi_r,   wr = p*w
%     psi_s_cm    = (Lm/Lr)*psi_r + sigma*Ls*i_s
%   Voltage model, corrected toward the current model:
%     d(psi_s)/dt = v - Rs*i_s - wc_cm*(psi_s - psi_s_cm)
%   Below ~wc_cm the estimate follows the current model, above it the
%   voltage model. The old leaky integrator (-wc_int*psi_s) pulled the
%   estimate to ZERO at standstill, which made DTC lose the flux when the
%   AGV stopped at a station.
pr = st.psr_cm;
wr = m.p*w_u;
st.psr_cm = pr + Ts*((Lm_c/tr_c)*i_ab - pr/tr_c + wr*[-pr(2); pr(1)]);
psis_cm = (Lm_c/Lr_c)*st.psr_cm + sLs_c*i_ab;
i_avg = 0.5*(i_ab + st.i_prev);
st.psis = st.psis + Ts*(st.v_app - m.Rs*i_avg - c.wc_cm*(st.psis - psis_cm));
Te = 1.5*m.p*(st.psis(1)*i_ab(2) - st.psis(2)*i_ab(1));
st.Te_est = Te;
% stator-flux reference from the rotor-flux reference
psr = max(st.psi_ref, 0.05*m.psi_r_rated);
iq = st.Te_ref/(1.5*m.p*(Lm_c/Lr_c)*psr);
psis_ref = sqrt((Ls_c/Lm_c*st.psi_ref)^2 + (sLs_c*iq)^2);
st.psis_ref = psis_ref;
st.id = i_ab(1);  st.iq = i_ab(2);  st.id_ref = 0;  st.iq_ref = iq;
% hysteresis comparators
psi_mag = norm(st.psis);
if psi_mag < psis_ref - c.h_psi, st.cpsi = 1; elseif psi_mag > psis_ref + c.h_psi, st.cpsi = 0; end
eT = st.Te_ref - Te;
if eT > c.h_T
    st.cT = 1;
elseif eT < -c.h_T
    st.cT = -1;
elseif (st.cT > 0.5 && eT <= 0) || (st.cT < -0.5 && eT >= 0)
    st.cT = 0;
end
% sector (1..6), sector 1 = -30..+30 deg
ang = atan2(st.psis(2), st.psis(1));
% 16 Oct 2026: if the flux estimate becomes NaN/Inf, sec is NaN and V(sec,:)
% reads outside the table in the JIT-compiled block (no bounds check there):
% MATLAB closed with an access violation. Keep the index inside 1..6.
if ~isfinite(ang), ang = 0; end
sec = mod(floor((ang + pi/6)/(pi/3)), 6) + 1;
sec = min(max(sec, 1), 6);
V = [1 0 0; 1 1 0; 0 1 0; 0 1 1; 0 0 1; 1 0 1];
if st.cT == 0 && psi_mag < 0.9*psis_ref
    s = V(sec, :)';                     % magnetising: radial vector builds flux
elseif st.cT == 0
    if sum(abs(st.legs - [0;0;0])) <= sum(abs(st.legs - [1;1;1])), s = [0;0;0]; else, s = [1;1;1]; end
else
    if st.cpsi > 0.5
        if st.cT > 0, kv = sec + 1; else, kv = sec - 1; end
    else
        if st.cT > 0, kv = sec + 2; else, kv = sec - 2; end
    end
    kv = min(max(mod(kv - 1, 6) + 1, 1), 6);
    s = V(kv, :)';
end
% torque limit is enforced through Te_ref; open phase: nothing else to remap
st.legs = s;
d = s;
end

% =====================================================================
function psi = flux_target(c, m, lut, mlp, w, T)
if c.flux_mode == 2
    psi = c.psi_fixed;
elseif c.flux_mode == 3
    psi = lut_bilinear(lut.w, lut.T, lut.psi, w, T);
elseif c.flux_mode == 4
    xn = ([w; T] - mlp.x_mu)./mlp.x_sd;
    y = mlp.W2*tanh(mlp.W1*xn + mlp.b1) + mlp.b2;
    psi = y*mlp.y_sd + mlp.y_mu;
else
    psi = m.psi_r_rated;
end
psi = min(max(psi, c.psi_min), m.psi_r_rated);
end

function z = lut_bilinear(xg, yg, Z, x, y)
nx = numel(xg);  ny = numel(yg);
x = min(max(x, xg(1)), xg(nx));
y = min(max(y, yg(1)), yg(ny));
i = 1;  while i < nx-1 && x > xg(i+1), i = i + 1; end
j = 1;  while j < ny-1 && y > yg(j+1), j = j + 1; end
tx = (x - xg(i))/(xg(i+1) - xg(i));
ty = (y - yg(j))/(yg(j+1) - yg(j));
z = (1-tx)*(1-ty)*Z(i,j) + tx*(1-ty)*Z(i+1,j) + (1-tx)*ty*Z(i,j+1) + tx*ty*Z(i+1,j+1);
end

function is = obs_currents(x, Ls, Lr, Lm)
D = Ls*Lr - Lm^2;
is = [(Lr*x(1) - Lm*x(3))/D; (Lr*x(2) - Lm*x(4))/D];
end

function x = obs_propagate(x, v, ib_m, wr, Ls, Lr, Lm, m, Ko, Ts)
% Open-loop alpha-beta machine model, corrected only with the phase-b sensor,
% so that its phase-a prediction is independent of the phase-a sensor.
D = Ls*Lr - Lm^2;
is = [(Lr*x(1) - Lm*x(3))/D; (Lr*x(2) - Lm*x(4))/D];
ir = [(Ls*x(3) - Lm*x(1))/D; (Ls*x(4) - Lm*x(2))/D];
eb = [-0.5; sqrt(3)/2];
ib_hat = eb'*is;
dx = zeros(4,1);
dx(1:2) = v - m.Rs*is + Ko*eb*(ib_m - ib_hat);
dx(3) = -m.Rr*ir(1) - wr*x(4);
dx(4) = -m.Rr*ir(2) + wr*x(3);
x = x + Ts*dx;
end
