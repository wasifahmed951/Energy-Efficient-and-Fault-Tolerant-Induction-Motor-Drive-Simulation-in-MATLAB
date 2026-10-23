function t_fault(ctype, ftype, remedy)
addpath('../params','../core','../sim');
P = default_params(ctype); P.ctrl.remedy_open = remedy;
scen = make_scenario('fault', P, struct('fault', ftype, 't_fault', 0.6, 'Tend', 0.9));
res = run_sim(P, scen, 'matlab');
I = log_index(); t = res.t;
kf = find(t >= 0.6, 1);
det = find(res.lc(:,I.c.f_speed)+res.lc(:,I.c.f_cur)+res.lc(:,I.c.f_open) > 0, 1);
if isempty(det), dt = NaN; else, dt = (t(det)-0.6)*1e3; end
pre = t>0.45 & t<0.6; post = t>0.75;
Tpp = @(ix) max(res.lp(ix,I.p.Te_max))-min(res.lp(ix,I.p.Te_min));
fprintf('%s %-7s remedy=%d | flags spd %d cur %d open %d neutral %d | detect %.1f ms | w pre %.1f post %.1f (ref %.1f) | Te pp pre %.1f post %.1f | Te mean post %.1f | max|ib| post %.1f\n', ...
  ctype, ftype, remedy, max(res.lc(:,I.c.f_speed)), max(res.lc(:,I.c.f_cur)), max(res.lc(:,I.c.f_open)), max(res.lc(:,I.c.neutral)), dt, ...
  mean(res.lp(pre,I.p.w)), mean(res.lp(post,I.p.w)), res.lc(end,I.c.w_ref), Tpp(pre), Tpp(post), mean(res.lp(post,I.p.Te)), max(abs(res.lp(post,I.p.ib))));
end
