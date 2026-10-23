function [psi_opt, Lopt, Lrated] = optimal_flux_analytic(w, T, P)
%OPTIMAL_FLUX_ANALYTIC  Rotor flux that minimises drive input power at (w, T).
%   Search range [P.ctrl.psi_min, rated]; infeasible points (current or
%   voltage limit) are penalised.
lo = P.ctrl.psi_min;  hi = P.m.psi_r_rated;
f = @(psi) cost(w, T, psi, P);
psi_opt = fminbnd(f, lo, hi, optimset('TolX', 1e-4));
Lopt = loss_model_ss(w, T, psi_opt, P);
Lrated = loss_model_ss(w, T, hi, P);
end

function c = cost(w, T, psi, P)
L = loss_model_ss(w, T, psi, P);
c = L.P_in;
if ~L.feasible, c = c + 1e4*(1 + max(L.I - P.m.I_max, 0)); end
end
