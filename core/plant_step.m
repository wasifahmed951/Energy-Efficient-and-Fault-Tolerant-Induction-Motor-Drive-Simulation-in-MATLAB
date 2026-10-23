function [meas, logp, pst] = plant_step(pst, u, ref, P)
%PLANT_STEP  Two-level inverter + saturated induction motor over one control period.
%
%   u   = [d_a; d_b; d_c; neutral_on]   leg duty cycles (0..1) and neutral contactor
%   ref = [w_ref; T_load; f_speed; f_cur; f_open; J_extra]
%         f_speed : speed-encoder fault (reading forced to 0)
%         f_cur   : phase-a current-sensor fault (gain forced to 0)
%         f_open  : phase a open-circuited
%         J_extra : extra inertia reflected to the shaft (AGV) [kg m^2]
%
%   Centre-aligned PWM with double update: in a "rising" half-period a leg
%   is off then on, in a "falling" half-period on then off. DTC passes 0/1
%   duties, which gives a constant voltage vector for the period.
%
%   The machine is integrated with RK4 over every constant-voltage segment,
%   so the PWM current and torque ripple are resolved exactly.
%
%   meas = [ia_meas; ib_meas; ic_meas; w_meas]     (after sensor faults)
%   logp = 23 logged quantities, see sim/log_index.m (I.p.*)

m = P.m;  iv = P.inv;  Ts = P.ctrl.Ts;  hmax = P.sim.h_max;  nit = P.sim.sat_iter;
Vdc = iv.Vdc;
TL = ref(2);  f_speed = ref(3) > 0.5;  f_cur = ref(4) > 0.5;  f_open = ref(5) > 0.5;
Jtot = m.J + ref(6);

% ---------- operating mode ----------
mode = 1;
if f_open
    if u(4) > 0.5, mode = 3; else, mode = 2; end
end
if mode ~= pst.mode
    if mode == 3
        % entering neutral-connected mode: keep i_alpha = 0 at the switch
        [~, ~, pm] = im_currents(pst.x, 0, pst.psim, m, nit);
        pst.x(1) = pm(1);
        pst.psim = pm;
    end
    pst.mode = mode;
end
if mode == 1
    ga = 1/m.Lls;  Ra = m.Rs;
elseif mode == 2
    ga = 0;        Ra = 0;
else
    ga = 1/(3*m.Lls);  Ra = 3*m.Rs;
end

% ---------- PWM segment break points ----------
d = min(max(u(1:3),0),1);
rising = (pst.half == 0);
tsw = zeros(3,1);
for j = 1:3
    if rising, tsw(j) = (1-d(j))*Ts; else, tsw(j) = d(j)*Ts; end
end
bp = sort([0; tsw; Ts]);

% ---------- integrate ----------
x = pst.x;  psim = pst.psim;
acc = zeros(12,1);  % [Te, Te^2, Pel, Pcond, Pcus, Pcur, ia, ia^2, Pout, Pfric, Psw(energy), nsw]
Te_min = 1e9;  Te_max = -1e9;
legs_prev = pst.legs;
for sgi = 1:4
    t0 = bp(sgi);  t1 = bp(sgi+1);  dt = t1 - t0;
    if dt <= 1e-12, continue; end
    tm = 0.5*(t0 + t1);
    s = zeros(3,1);
    for j = 1:3
        if rising, s(j) = double(tm >= tsw(j)); else, s(j) = double(tm < tsw(j)); end
    end
    vp = (s - 0.5)*Vdc;                          % pole voltages w.r.t. DC mid-point
    % state derivative and outputs at the start of the segment
    [k1, o1, psim] = im_deriv(x, vp, TL, Jtot, ga, Ra, mode, psim, m, iv, nit);
    % switching events at the start of this segment
    for j = 1:3
        if s(j) ~= legs_prev(j)
            acc(11) = acc(11) + iv.Esw*(Vdc/iv.Vref)*(abs(o1(5+j))/iv.Iref);
            acc(12) = acc(12) + 1;
        end
    end
    legs_prev = s;
    Te_min = min(Te_min, o1(1));  Te_max = max(Te_max, o1(1));
    nsub = max(1, ceil(dt/hmax - 1e-9));
    h = dt/nsub;
    for k = 1:nsub
        [k2, ~,  psim] = im_deriv(x + 0.5*h*k1, vp, TL, Jtot, ga, Ra, mode, psim, m, iv, nit);
        [k3, ~,  psim] = im_deriv(x + 0.5*h*k2, vp, TL, Jtot, ga, Ra, mode, psim, m, iv, nit);
        [k4, ~,  psim] = im_deriv(x + h*k3,     vp, TL, Jtot, ga, Ra, mode, psim, m, iv, nit);
        xn = x + (h/6)*(k1 + 2*k2 + 2*k3 + k4);
        % end-of-substep evaluation: trapezoid rule for the logged integrals,
        % and it is the first RK4 stage of the next substep
        [k1n, o1n, psim] = im_deriv(xn, vp, TL, Jtot, ga, Ra, mode, psim, m, iv, nit);
        Te_min = min(Te_min, o1n(1));  Te_max = max(Te_max, o1n(1));
        f0 = [o1(1); o1(1)^2; o1(2); o1(3); o1(4); o1(5); o1(6); o1(6)^2; TL*x(5); m.B*x(5)^2];
        f1 = [o1n(1); o1n(1)^2; o1n(2); o1n(3); o1n(4); o1n(5); o1n(6); o1n(6)^2; TL*xn(5); m.B*xn(5)^2];
        acc(1:10) = acc(1:10) + 0.5*h*(f0 + f1);
        % 17 Oct 2026: Te^2 and ia^2 (rms torque ripple, total THD) are integrated exactly
        % for a straight line between the two end points, int f^2 = h/3*(f0^2 + f0*f1 + f1^2).
        % Between switching instants the ripple is a ramp; the trapezoid on f^2 used before
        % over-estimated its mean square (rms ripple about 13 % too high).
        acc(2) = acc(2) - 0.5*h*(o1(1)^2 + o1n(1)^2) + (h/3)*(o1(1)^2 + o1(1)*o1n(1) + o1n(1)^2);
        acc(8) = acc(8) - 0.5*h*(o1(6)^2 + o1n(6)^2) + (h/3)*(o1(6)^2 + o1(6)*o1n(6) + o1n(6)^2);
        x = xn;  k1 = k1n;  o1 = o1n;
    end
end
pst.legs = legs_prev;
pst.half = 1 - pst.half;
pst.x = x;  pst.psim = psim;

% ---------- end-of-step outputs ----------
[Te_e, ip, ~, psim] = plant_out(x, ga, psim, mode, m, nit);
pst.psim = psim;
Te_min = min(Te_min, Te_e);  Te_max = max(Te_max, Te_e);
psim_mag = sqrt(psim(1)^2 + psim(2)^2);
ang = atan2(psim(2), psim(1));
dang = ang - pst.ang_m;
dang = dang - 2*pi*round(dang/(2*pi));
fe = dang/(2*pi*Ts);
pst.ang_m = ang;
Pfe = (m.kh*abs(fe) + m.ke*fe^2) * psim_mag^2;
psir_mag = sqrt(x(3)^2 + x(4)^2);

meas = [ip(1)*(1 - double(f_cur)); ip(2); ip(3); x(5)*(1 - double(f_speed))];

logp = zeros(23,1);
logp(1:3) = ip;
logp(4)  = x(5);
logp(5)  = acc(1)/Ts;            % Te average
logp(6)  = Te_min;
logp(7)  = Te_max;
logp(8)  = acc(2)/Ts;            % Te^2 average
logp(9)  = acc(3)/Ts;            % electrical power into the motor
logp(10) = acc(4)/Ts;            % inverter conduction loss
logp(11) = acc(11)/Ts;           % inverter switching loss
logp(12) = Pfe;                  % core loss
logp(13) = acc(5)/Ts;            % stator copper loss
logp(14) = acc(6)/Ts;            % rotor copper loss
logp(15) = psir_mag;
logp(16) = psim_mag;
logp(17) = acc(7)/Ts;            % ia average over the step
logp(18) = acc(8)/Ts;            % ia^2 average over the step
logp(19) = acc(12);              % number of leg transitions in this step
logp(20) = mode;
logp(21) = TL;
logp(22) = acc(9)/Ts;            % load (output) power
logp(23) = acc(10)/Ts;           % friction loss
end

% =====================================================================
function [dx, o, psim] = im_deriv(x, vp, TL, Jtot, ga, Ra, mode, psim, m, iv, nit)
% State derivative. o = [Te; Pel; Pcond; Pcus; Pcur; ia; ib; ic]
[is, ir, psim] = im_currents(x, ga, psim, m, nit);
vb = (vp(2) - vp(3))/sqrt(3);
if mode == 1
    va = (2/3)*(vp(1) - 0.5*(vp(2) + vp(3)));
elseif mode == 3
    va = -(vp(2) + vp(3));
else
    va = 0;
end
wr = m.p*x(5);
Te = 1.5*m.p*(psim(1)*is(2) - psim(2)*is(1));
dx = zeros(6,1);
if mode ~= 2
    dx(1) = va - Ra*is(1);
end
dx(2) = vb - m.Rs*is(2);
dx(3) = -m.Rr*ir(1) - wr*x(4);
dx(4) = -m.Rr*ir(2) + wr*x(3);
dx(5) = (Te - TL - m.B*x(5))/Jtot;
dx(6) = x(5);
ip = phase_currents(is, mode);
Pel = vp(1)*ip(1) + vp(2)*ip(2) + vp(3)*ip(3);
Pcond = 0;
for j = 1:3
    Pcond = Pcond + iv.Vce0*abs(ip(j)) + iv.rce*ip(j)^2;
end
o = [Te; Pel; Pcond; m.Rs*(ip(1)^2 + ip(2)^2 + ip(3)^2); 1.5*m.Rr*(ir(1)^2 + ir(2)^2); ip(1); ip(2); ip(3)];
end

function [Te, ip, is, psim] = plant_out(x, ga, psim, mode, m, nit)
[is, ~, psim] = im_currents(x, ga, psim, m, nit);
Te = 1.5*m.p*(psim(1)*is(2) - psim(2)*is(1));
ip = phase_currents(is, mode);
end

function ip = phase_currents(is, mode)
% Phase currents from alpha-beta (amplitude-invariant Clarke)
s3 = sqrt(3)/2;
if mode == 1
    ip = [is(1); -0.5*is(1) + s3*is(2); -0.5*is(1) - s3*is(2)];
elseif mode == 2
    ip = [0; s3*is(2); -s3*is(2)];
else
    % neutral connected: i0 = -i_alpha so that ia = 0
    ip = [0; -1.5*is(1) + s3*is(2); -1.5*is(1) - s3*is(2)];
end
end
