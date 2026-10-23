%% diag_agv2.m  -  why do some AGV missions misbehave? (16 Oct 2026)
% Section 9 of main_run_all showed:
%   * healthy missions with LUT flux (flux_mode 3) still go to limp mode
%     (IFOC at 37.7 s, DTC at 25.2 s) - a FALSE fault alarm; DTC then uses 15.9 Wh vs 7.5 Wh
%   * every speed-sensor mission (fault 1) fails; IFOC stops at s = 18.56 m = station B
% This runs those missions with the gains of main_run_all (results/tuned_params.mat)
% and prints which flag fired, when, and what speed/current looked like.
% Run:  diag_agv2      (prints the diagnosis of each mission).

setup_project;
S = load(fullfile('results', 'tuned_params.mat'), 'Pi_best', 'Pd_best');
I = log_index();
cases = { 'IFOC', 3, 0;          % false alarm, LUT flux
          'DTC',  3, 0;          % false alarm + high energy
          'IFOC', 1, 1;          % speed-sensor fault, stuck at B
          'DTC',  3, 2 };        % crashed MATLAB before the NaN guard: does it diverge?
flags = {'f_speed', 'f_cur', 'f_open', 'neutral'};
for k = 1:size(cases, 1)
    [ty, fm, fault] = cases{k, :};
    if strcmp(ty, 'IFOC'), P = S.Pi_best; else, P = S.Pd_best; end
    P.ctrl.flux_mode = fm;  P.agv.fault_type = fault;
    fprintf('\n%s  ==== %s  flux %d  fault %d ====\n', datestr(now, 'HH:MM:SS'), ty, fm, fault);
    tic;  res = run_agv(P, 'simulink');  fprintf('sim took %.0f s\n', toc);
    t = res.t;  la = res.la;  M = res.M;
    fprintf('completed %d, time %.1f s, %.2f Wh, s_end %.2f m, limp %d, stop %d, t_detect %.3f\n', ...
        M.completed, M.mission_time, M.E_Wh, la(end,14), M.limp, M.stopped, M.t_detect);
    if M.aborted
        kn = find(~isfinite(res.lpL(:, I.p.w)) | ~isfinite(res.lpR(:, I.p.w)) | ~isfinite(la(:, 7)), 1);
        if isempty(kn), kn = numel(t); end
        fprintf('  ABORTED: drive diverged, first non-finite value at %.3f s\n', t(kn));
    end
    ph = la(:,10);
    for p = 2:4
        kk = find(ph >= p, 1);
        if isempty(kk), fprintf('  phase %d never reached\n', p); else, fprintf('  phase %d at %.2f s\n', p, t(kk)); end
    end
    sides = {'L', res.lpL, res.lcL; 'R', res.lpR, res.lcR};
    for sd = 1:2
        lp = sides{sd, 2};  lc = sides{sd, 3};
        for f = 1:4
            kf = find(lc(:, 10 + f) > 0.5, 1);
            if isempty(kf), continue; end
            w = max(1, kf - 5):min(numel(t), kf + 5);
            fprintf('  %s: %s set at %.4f s | w_ref %.1f  w_true %.1f  w_est %.1f  ia %.2f  ia_hat %.2f  psi_ref %.3f  Te_ref %.2f\n', ...
                sides{sd, 1}, flags{f}, t(kf), lc(kf, I.c.w_ref), lp(kf, I.p.w), lc(kf, I.c.w_est), ...
                lp(kf, I.p.ia), lc(kf, I.c.ia_hat), lc(kf, I.c.psi_ref), lc(kf, I.c.Te_ref));
            fprintf('      around it: max|ia - ia_hat| %.2f A, max|w - w_est| %.1f rad/s, psi_ref %.3f..%.3f\n', ...
                max(abs(lp(w, I.p.ia) - lc(w, I.c.ia_hat))), max(abs(lp(w, I.p.w) - lc(w, I.c.w_est))), ...
                min(lc(w, I.c.psi_ref)), max(lc(w, I.c.psi_ref)));
        end
        Pin = sum(lp(:, 9:12), 2);
        fprintf('  %s: mean Pin %.0f W, max |ia| %.1f A, max |w| %.1f rad/s, max|w - w_est| %.1f rad/s\n', sides{sd, 1}, ...
            mean(Pin), max(abs(lp(:, I.p.ia))), max(abs(lp(:, I.p.w))), max(abs(lp(:, I.p.w) - lc(:, I.c.w_est))));
    end
    % at and after station B: is the speed estimate usable at standstill?
    kb = find(ph >= 2, 1);
    if ~isempty(kb)
        w = kb:min(numel(t), kb + round(6/mean(diff(t))));
        fprintf('  6 s from reaching B: right w_true %.1f..%.1f, w_est %.1f..%.1f, w_used %.1f..%.1f, |v| max %.3f\n', ...
            min(res.lpR(w, I.p.w)), max(res.lpR(w, I.p.w)), min(res.lcR(w, I.c.w_est)), max(res.lcR(w, I.c.w_est)), ...
            min(res.lcR(w, I.c.w_used)), max(res.lcR(w, I.c.w_used)), max(abs(la(w, 4))));
    end
    figure('Name', sprintf('%s flux %d fault %d', ty, fm, fault), 'Color', 'w');
    subplot(4,1,1); plot(la(:,1), la(:,2)); axis equal; grid on; title(sprintf('%s flux %d fault %d: path', ty, fm, fault));
    subplot(4,1,2); plot(t, res.lpR(:, I.p.w), t, res.lcR(:, I.c.w_est), t, res.lcR(:, I.c.w_ref)); grid on;
    legend('w true R', 'w est R', 'w ref R'); ylabel('rad/s');
    subplot(4,1,3); plot(t, ph, t, la(:,11), '--', t, res.lcR(:,11) + 0.1, t, res.lcR(:,12) + 0.2, t, res.lcL(:,12) + 0.3); grid on;
    legend('phase', 'limp', 'f\_speed R', 'f\_cur R', 'f\_cur L'); ylim([-0.2 4.5]);
    subplot(4,1,4); plot(t, res.lcR(:, I.c.psi_ref), t, res.lcL(:, I.c.psi_ref)); grid on; ylabel('\psi_{ref}'); xlabel('t (s)');
    clear res
end
fprintf('\nDone.\n');
