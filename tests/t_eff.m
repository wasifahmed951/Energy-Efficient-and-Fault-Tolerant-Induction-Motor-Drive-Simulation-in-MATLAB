addpath('../params','../core','../sim','../analysis','../efficiency','../utils');
P = default_params('IFOC'); wr = P.m.wm_rated; Tr = P.m.T_rated;
L = loss_model_ss(0.5*wr, 0.25*Tr, 0.95, P);
fprintf('analytic @0.5wr,0.25Tr,rated flux: Pin %.0f Pout %.0f Pfe %.0f Pcus %.0f Pcur %.0f Psw %.0f Pcond %.0f\n', L.P_in, L.P_out, L.P_fe, L.P_cus, L.P_cur, L.P_sw, L.P_cond);
for wt = [0.3 0.6 0.9; 0.25 0.25 0.25]
  for TT = [0.25 0.5]
   [po, Lo, Lr] = optimal_flux_analytic(wt(1)*wr, TT*Tr, P);
   fprintf('w=%.1f T=%.2f: psi_opt %.3f  Pin %.0f -> %.0f  (saving %.1f%%)\n', wt(1), TT, po, Lr.P_in, Lo.P_in, (Lr.P_in-Lo.P_in)/Lr.P_in*100);
  end
end
tic; P = build_flux_lut(P, 'analytic'); toc
disp(P.lut.psi(1:3:10,1:3:10))
save('-mat','P_lut.mat','P');
