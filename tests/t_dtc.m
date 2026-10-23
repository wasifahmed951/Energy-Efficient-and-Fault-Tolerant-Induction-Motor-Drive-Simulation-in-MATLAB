addpath('../params','../core','../sim');
P = default_params('DTC');
scen = make_scenario('tuning', P);
tic; res = run_sim(P, scen, 'matlab'); toc
I = log_index(); t = res.t; w = res.lp(:,I.p.w); Te = res.lp(:,I.p.Te);
for tt = [0.15 0.25 0.3 0.4 0.6 0.69 0.72 0.8 0.99]
  k = find(t>=tt,1);
  fprintf('t=%.2f w=%7.2f (ref %6.2f) Te=%6.2f Teref=%6.2f psir=%.3f psiref=%.3f id=%.2f/%.2f iq=%.2f/%.2f west=%.1f\n', ...
    tt, w(k), res.lc(k,I.c.w_ref), Te(k), res.lc(k,I.c.Te_ref), res.lp(k,I.p.psir), res.lc(k,I.c.psi_ref), ...
    res.lc(k,I.c.id), res.lc(k,I.c.id_ref), res.lc(k,I.c.iq), res.lc(k,I.c.iq_ref), res.lc(k,I.c.w_est));
end
fprintf('flags: speed %d cur %d open %d\n', max(res.lc(:,I.c.f_speed)), max(res.lc(:,I.c.f_cur)), max(res.lc(:,I.c.f_open)));
fprintf('max |ia - ia_hat| after 0.3 s: %.2f A\n', max(abs(res.lp(t>0.3,I.p.ia)-res.lc(t>0.3,I.c.ia_hat))));
save('-mat','res_dtc.mat','res');
idx = t>0.55 & t<0.7; fsw = sum(res.lp(idx,I.p.nsw))/3/(sum(idx)*P.ctrl.Ts)/2;
fprintf('avg device switching freq (0.55-0.7 s): %.0f Hz; Te ripple pp %.2f N m\n', fsw, max(res.lp(idx,I.p.Te_max))-min(res.lp(idx,I.p.Te_min)));
