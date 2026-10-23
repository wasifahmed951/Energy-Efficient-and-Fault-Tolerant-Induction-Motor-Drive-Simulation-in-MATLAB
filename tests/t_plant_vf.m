1;
function a = cumf(f,t)
  % integral of the ramp frequency
  if t < 0.5, a = 50*t^2/(2*0.5); else, a = 50*0.5/2 + 50*(t-0.5); end
end

% Open-loop V/f start of the plant: checks speed, current and power balance.
addpath('../params','../core');
P = default_params('IFOC'); Ts = P.ctrl.Ts;
pst = plant_init(P);
N = round(1.2/Ts); w = zeros(N,1); ia = w; Te = w; Pel = w; Pl=w;
for k = 1:N
  t = (k-1)*Ts;
  f = min(50, 50*t/0.5);                 % ramp to 50 Hz in 0.5 s
  V = 326.6*max(f/50, 0.04);             % phase peak voltage (V/f)
  th = 2*pi*cumf(f,t);
  v = V*[cos(th); cos(th-2*pi/3); cos(th+2*pi/3)];
  d = 0.5 + v/P.inv.Vdc;
  TL = (t>0.8)*P.m.T_rated;
  [meas, logp, pst] = plant_step(pst, [d;0], [0;TL;0;0;0;0], P);
  w(k) = logp(4); ia(k)=logp(1); Te(k)=logp(5); Pel(k)=logp(9); Pl(k)=logp(22);
end
fprintf('speed at 0.75 s: %.1f rpm (sync 1500)\n', w(round(0.75/Ts))*60/2/pi);
fprintf('speed at 1.2 s (rated load): %.1f rpm (rated 1430)\n', w(end)*60/2/pi);
idx = round(1.0/Ts):N;
fprintf('ia peak at rated load: %.2f A, mean Te %.2f N m (rated %.2f)\n', max(abs(ia(idx))), mean(Te(idx)), P.m.T_rated);
fprintf('Pel %.0f W, Pload %.0f W, eta_motor(no core) %.3f\n', mean(Pel(idx)), mean(Pl(idx)), mean(Pl(idx))/mean(Pel(idx)));
