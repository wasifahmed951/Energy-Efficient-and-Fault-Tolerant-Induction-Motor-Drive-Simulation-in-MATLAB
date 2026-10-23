function [is, ir, psim] = im_currents(x, ga, psim0, m, n_iter)
%IM_CURRENTS  Stator/rotor currents from flux states, with saturation.
%   x     = [lam_a; psi_sb; psi_ra; psi_rb; ...]  (flux states)
%   ga    = 1/(alpha-axis stator leakage); 0 when phase a is open with an
%           isolated neutral (i_alpha forced to zero)
%   psim0 = warm-start air-gap flux [2x1]
%   Returns is = [i_alpha; i_beta], ir = [i_ra; i_rb], psim = [psi_ma; psi_mb].
%
%   Uses  (1/Lms + g_s + g_r) * psi_m = g_s*psi_s + g_r*psi_r  per axis,
%   solved by fixed-point iteration on the secant inductance Lms(|psi_m|).

gb = 1/m.Lls;
gr = 1/m.Llr;
ua = ga*x(1) + gr*x(3);
ub = gb*x(2) + gr*x(4);
psim = psim0;
for k = 1:n_iter
    L = sat_Lms(sqrt(psim(1)^2 + psim(2)^2), m);
    psim = [ua/(1/L + ga + gr); ub/(1/L + gb + gr)];
end
is = [ga*(x(1) - psim(1)); gb*(x(2) - psim(2))];
ir = [gr*(x(3) - psim(1)); gr*(x(4) - psim(2))];
end
