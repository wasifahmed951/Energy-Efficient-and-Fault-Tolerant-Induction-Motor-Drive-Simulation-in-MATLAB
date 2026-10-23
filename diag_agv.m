%% diag_agv.m  -  why do the DTC AGV missions not complete?
% Runs ONE healthy mission (rated flux, no fault) for IFOC and for DTC and
% prints where the mission gets stuck. Takes about 1-3 minutes.
% Put this file in the main "Electrical Drive" folder and run:  diag_agv
%
% NOTE: speed-PI gains below are the ZN values. If main_run_all uses the
% GA/PSO gains for section 9, put those numbers here instead.

setup_project;                                   % adds all folders to the path
gains = struct('IFOC', [7.46 1993], 'DTC', [7.95 2509]);
types = {'IFOC', 'DTC'};
R = struct();

for k = 1:2
    ty = types{k};
    P = default_params(ty);
    P.ctrl.Kp_w = gains.(ty)(1);
    P.ctrl.Ki_w = gains.(ty)(2);
    P.ctrl.flux_mode = 1;
    P.agv.fault_type = 0;
    fprintf('\nRunning %s healthy mission ...\n', ty);
    res = run_agv(P, 'simulink');
    R.(ty) = res;

    la = res.la;  t = res.t;  M = res.M;
    ph = la(:,10);
    tph = @(p) t(find(ph >= p, 1));
    fprintf('==== %s ====\n', ty);
    fprintf('completed           : %d   (sim ended at t = %.1f s)\n', M.completed, t(end));
    fprintf('reached B (phase 2) : %s\n', num2str(tph(2)));
    fprintf('left B   (phase 3)  : %s\n', num2str(tph(3)));
    fprintf('final phase         : %d,  path progress s = %.2f of %.1f m\n', ph(end), la(end,14), M.path_length_m);
    fprintf('limp flag / stop flag ever set : %d / %d   (fault report time %.2f s)\n', ...
        any(la(:,11) > 0.5), any(la(:,12) > 0.5), la(end,16));
    w = t > t(end) - 5;
    fprintf('last 5 s: mean|v| = %.3f m/s, max|v| = %.3f m/s, v_cmd(end) = %.3f m/s\n', ...
        mean(abs(la(w,4))), max(abs(la(w,4))), la(end,6));
    fprintf('energy %.2f Wh;  mean battery power %.0f W\n', la(end,7), la(end,7)*3600/t(end));
    % mean input power of both drives in each phase
    Pin = sum(res.lpL(:,9:12), 2) + sum(res.lpR(:,9:12), 2);
    for p = 1:4
        m = ph == p;
        if any(m), fprintf('  phase %d: %5.1f s, mean input power %6.0f W\n', p, sum(m)*mean(diff(t)), mean(Pin(m))); end
    end
    fprintf('controller fault flags (left / right, cols 11-14): [%s] / [%s]\n', ...
        num2str(max(res.lcL(:,11:14), [], 1)), num2str(max(res.lcR(:,11:14), [], 1)));

    figure('Name', ['AGV diagnosis - ' ty], 'Color', 'w');
    subplot(4,1,1); plot(la(:,1), la(:,2)); axis equal; grid on; title([ty ': x-y path']);
    subplot(4,1,2); plot(t, la(:,4), t, la(:,6)); grid on; legend('v', 'v_{cmd}'); ylabel('m/s');
    subplot(4,1,3); plot(t, ph, t, la(:,11), '--', t, la(:,12), ':'); grid on;
    legend('phase', 'limp', 'stop'); ylim([-0.2 4.5]);
    subplot(4,1,4); plot(t, Pin); grid on; ylabel('P_{in} both drives (W)'); xlabel('t (s)');
end
save(fullfile('results', 'diag_agv.mat'), 'R', '-v7.3');
fprintf('\nDone. Diagnosis printed above; data saved in results/diag_agv.mat.\n');
