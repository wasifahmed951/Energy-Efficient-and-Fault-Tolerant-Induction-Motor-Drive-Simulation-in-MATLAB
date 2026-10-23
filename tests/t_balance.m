function t_balance(sat, hmax, nit)
addpath('../params','../core','../sim');
P = default_params('IFOC'); P.m.sat_on = sat; P.sim.h_max = hmax; P.sim.sat_iter = nit;
Ts = P.ctrl.Ts; pst = plant_init(P); N = round(0.9/Ts); A = zeros(N,23);
for k = 1:N
  t = (k-1)*Ts; f = min(50, 50*t/0.4); V = 326.6*max(f/50,0.04);
  if t < 0.4, th = 2*pi*50*t^2/0.8; else, th = 2*pi*(50*0.2 + 50*(t-0.4)); end
  d = 0.5 + V*[cos(th); cos(th-2*pi/3); cos(th+2*pi/3)]/P.inv.Vdc;
  [~, lp, pst] = plant_step(pst, [d;0], [0;0.25*P.m.T_rated;0;0;0;0], P); A(k,:) = lp';
end
p = mean(A(round(0.7/Ts):end,:));
fprintf('sat=%d h=%g nit=%d: Pel %.1f  out+losses %.1f  diff %.1f W\n', sat, hmax, nit, p(9), p(22)+p(13)+p(14)+p(23), p(9)-(p(22)+p(13)+p(14)+p(23)));
end
