function [P, info] = run_zn_relay(P, engine)
%RUN_ZN_RELAY  Ziegler-Nichols speed-PI tuning from a relay-feedback test.
%   The speed PI is replaced by a relay (+/- relay_T around the load torque),
%   the speed oscillates, and from its amplitude a and period Pu:
%       Ku = 4*relay_T/(pi*a),  Kp = 0.45*Ku,  Ki = 0.54*Ku/Pu   (ZN PI rule)
%   IFOC current loops keep the pole-zero-cancellation design of default_params.
if nargin < 2, engine = 'simulink'; end
Pr = P;
Pr.ctrl.speed_mode = 2;
T_load = 0.25*P.m.T_rated;
Pr.ctrl.relay_bias = T_load + P.m.B*0.5*P.m.wm_rated;
scen = make_scenario('steady', Pr, struct('w', 0.5*P.m.wm_rated, 'T', T_load, 'Tend', 1.6));
res = run_sim(Pr, scen, engine);
I = log_index();
ix = res.t > 0.9;
w = res.lp(ix, I.p.w);  t = res.t(ix);
a = (max(w) - min(w))/2;
wc = w - mean(w);
zc = find(wc(1:end-1) < 0 & wc(2:end) >= 0);
Pu = mean(diff(t(zc)));
Ku = 4*Pr.ctrl.relay_T/(pi*a);
P.ctrl.Kp_w = 0.45*Ku;
P.ctrl.Ki_w = 0.54*Ku/Pu;
info = struct('a', a, 'Pu', Pu, 'Ku', Ku, 'Kp_w', P.ctrl.Kp_w, 'Ki_w', P.ctrl.Ki_w, 'res', res);
fprintf('ZN relay test: a = %.3f rad/s, Pu = %.4f s, Ku = %.3f -> Kp_w = %.4f, Ki_w = %.3f\n', ...
    a, Pu, Ku, P.ctrl.Kp_w, P.ctrl.Ki_w);
end
