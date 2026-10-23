%CHECK_INSTALL  Run this first on your PC (about 5-10 minutes).
%   1. builds both Simulink models,
%   2. runs IFOC and DTC on the tuning cycle with the Simulink AND the
%      plain-MATLAB engine and checks they agree,
%   3. runs a short open-phase fault case,
%   4. runs a short AGV mission in Simulink.
%   Every step prints PASS/FAIL. If a Simulink step fails, the MATLAB
%   engine result tells you whether the problem is in the model or the code.
clear; clc;
setup_project();
I = log_index();
ok_all = true;

fprintf('\n[1] Building Simulink models...\n');
try
    build_drive_model('IM_DRIVE');
    build_agv_model('AGV_MISSION');
    fprintf('    PASS\n');
catch err
    fprintf('    FAIL: %s\n', err.message);  ok_all = false;
end

for drv = {'IFOC', 'DTC'}
    fprintf('\n[2] %s on the tuning cycle: Simulink vs MATLAB engine\n', drv{1});
    P = default_params(drv{1});
    sc = make_scenario('tuning', P);
    rm = run_sim(P, sc, 'matlab');
    try
        rs = run_sim(P, sc, 'simulink');
        n = min(size(rs.lp,1), size(rm.lp,1));
        dw = max(abs(rs.lp(1:n, I.p.w) - rm.lp(1:n, I.p.w)));
        Ms = compute_metrics(rs);
        fprintf('    max speed difference between engines: %.2e rad/s\n', dw);
        fprintf('    rise %.3f s, overshoot %.1f %%, ripple %.1f %%, fsw %.0f Hz, eta %.3f\n', ...
            Ms.rise_time, Ms.overshoot, Ms.Te_ripple_pp, Ms.fsw_avg, Ms.eta_drive);
        if dw < 1e-3*P.m.wm_rated, fprintf('    PASS\n'); else, fprintf('    FAIL (engines differ)\n'); ok_all = false; end
    catch err
        fprintf('    FAIL (Simulink): %s\n', err.message);  ok_all = false;
        Mm = compute_metrics(rm);
        fprintf('    MATLAB engine alone: rise %.3f s, eta %.3f\n', Mm.rise_time, Mm.eta_drive);
    end
end

fprintf('\n[3] IFOC open phase with neutral remedy (Simulink)\n');
try
    P = default_params('IFOC');
    r = run_sim(P, make_scenario('fault', P, struct('fault', 'open')), 'simulink');
    M = compute_metrics(r);
    fprintf('    detected after %.1f ms, post-fault speed error %.2f %%\n', M.t_detect, M.post_speed_err_pct);
    if M.t_detect < 20 && M.post_speed_err_pct < 2, fprintf('    PASS\n'); else, fprintf('    CHECK\n'); end
catch err
    fprintf('    FAIL: %s\n', err.message);  ok_all = false;
end

fprintf('\n[4] Short AGV mission (Simulink)\n');
try
    P = default_params('IFOC');
    P.agv.Lx = 3;  P.agv.Ly = 1.5;  P.agv.Tend = 30;  P.agv.dwell = 1;
    r = run_agv(P, 'simulink');
    fprintf('    completed %d in %.1f s, %.2f Wh, max cross-track error %.1f cm\n', ...
        r.M.completed, r.M.mission_time, r.M.E_Wh, r.M.e_cross_max_cm);
    if r.M.completed && ~r.M.limp, fprintf('    PASS\n'); else, fprintf('    CHECK\n'); end
catch err
    fprintf('    FAIL: %s\n', err.message);  ok_all = false;
end

if ok_all, fprintf('\nAll checks passed. Continue with main_run_all.m\n');
else, fprintf('\nSome checks failed - see messages above.\n'); end
