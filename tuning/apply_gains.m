function P = apply_gains(P, x)
%APPLY_GAINS  Put a decision vector (log10 of the gains) into P.
%   IFOC: x = log10([Kp_w Ki_w Kp_i Ki_i]);  DTC: x = log10([Kp_w Ki_w])
g = 10.^x(:)';
P.ctrl.Kp_w = g(1);  P.ctrl.Ki_w = g(2);
if numel(g) >= 4
    P.ctrl.Kp_i = g(3);  P.ctrl.Ki_i = g(4);
end
end
