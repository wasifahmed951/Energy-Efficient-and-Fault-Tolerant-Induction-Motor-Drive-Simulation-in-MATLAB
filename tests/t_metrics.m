addpath('../params','../core','../sim','../analysis');
for f = {'res_ifoc.mat','res_dtc.mat'}
  S = load(f{1}); M = compute_metrics(S.res);
  fprintf('%s: rise %.3f s, OS %.1f%%, settle %.3f s, ITAE %.3f, dip %.0f rpm, rec %.3f s\n  ripple pp %.1f%% rms %.2f%%, THD50 %.2f%%, THDtot %.2f%%, fsw %.0f Hz\n  Pin %.0f Pout %.0f eta_drive %.3f eta_motor %.3f (Pfe %.0f Psw %.0f Pcond %.0f)\n', ...
   f{1}, M.rise_time, M.overshoot, M.settling_time, M.itae, M.load_dip_rpm, M.load_recovery, M.Te_ripple_pp, M.Te_ripple_rms, M.thd50, M.thd_total, M.fsw_avg, M.P_in, M.P_out, M.eta_drive, M.eta_motor, M.P_fe, M.P_sw, M.P_cond);
end
