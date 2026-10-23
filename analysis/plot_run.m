function plot_run(res, ttl)
%PLOT_RUN  Speed, torque, flux and phase current of one run.
if nargin < 2, ttl = ''; end
I = log_index();  t = res.t;
figure('Name', ttl, 'Color', 'w');
subplot(4,1,1);
plot(t, res.lc(:, I.c.w_ref)*30/pi, '--', t, res.lp(:, I.p.w)*30/pi);
ylabel('speed [rpm]'); legend('reference', 'actual', 'Location', 'best'); title(ttl); grid on;
subplot(4,1,2);
plot(t, res.lc(:, I.c.Te_ref), '--', t, res.lp(:, I.p.Te));
ylabel('torque [N m]'); legend('reference', 'actual (step average)', 'Location', 'best'); grid on;
subplot(4,1,3);
plot(t, res.lc(:, I.c.psi_ref), '--', t, res.lp(:, I.p.psir));
ylabel('flux [Wb]'); legend('reference', 'rotor flux', 'Location', 'best'); grid on;
subplot(4,1,4);
plot(t, res.lp(:, [I.p.ia I.p.ib I.p.ic]));
ylabel('current [A]'); xlabel('time [s]'); legend('i_a', 'i_b', 'i_c', 'Location', 'best'); grid on;
end
