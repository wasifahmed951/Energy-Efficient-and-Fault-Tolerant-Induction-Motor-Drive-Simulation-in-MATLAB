function L = loss_model_ss(w, T, psir, P)
%LOSS_MODEL_SS  Steady-state losses of the drive at speed w [rad/s], load
%   torque T [N m] and rotor flux psir [Wb], with magnetic saturation.
%   Same machine and loss coefficients as the Simulink plant, so the
%   analytic optimum can be checked against the simulated sweep.
%
%   L.P_in, P_out, P_cus, P_cur, P_fe, P_fric, P_cond, P_sw [W], L.feasible
m = P.m;  iv = P.inv;
Te = T + m.B*w;
psim = psir;
for it = 1:6
    Lm = sat_Lms(psim, m);  Lr = m.Llr + Lm;
    iq = Te/(1.5*m.p*(Lm/Lr)*psir);
    id = psir/Lm;
    psim = sqrt(psir^2 + (m.Llr*(Lm/Lr)*iq)^2);   % air-gap flux = psi_r - Llr*i_r
end
Ls = m.Lls + Lm;  sLs = Ls - Lm^2/Lr;
w_sl = m.Rr*Lm*iq/(Lr*psir);
we = m.p*w + w_sl;  fe = we/(2*pi);
L.P_out  = T*w;
L.P_cus  = 1.5*m.Rs*(id^2 + iq^2);
L.P_cur  = 1.5*m.Rr*(Lm/Lr)^2*iq^2;
L.P_fe   = (m.kh*abs(fe) + m.ke*fe^2)*psim^2;
L.P_fric = m.B*w^2;
Ipk = sqrt(id^2 + iq^2);
L.P_cond = 3*(iv.Vce0*2*Ipk/pi + iv.rce*Ipk^2/2);
L.P_sw   = 3*iv.fsw*iv.Esw*(iv.Vdc/iv.Vref)*(2*Ipk/pi)/iv.Iref;
L.P_in   = L.P_out + L.P_cus + L.P_cur + L.P_fe + L.P_fric + L.P_cond + L.P_sw;
vd = m.Rs*id - we*sLs*iq;
vq = m.Rs*iq + we*(sLs*id + (Lm/Lr)*psir);
L.V = sqrt(vd^2 + vq^2);
L.I = Ipk;
L.feasible = (Ipk <= m.I_max) && (L.V <= 0.95*iv.Vdc/sqrt(3));
L.id = id;  L.iq = iq;  L.Lm = Lm;  L.psim = psim;
end
