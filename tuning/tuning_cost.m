function [J, M] = tuning_cost(x, P, engine)
%TUNING_COST  Common cost for ZN, GA and PSO (proposal section 5.3):
%   J = ITAE + w1*overshoot[%] + w2*rms torque ripple[% of rated]
%   on the 'tuning' cycle (speed step + load step). Unstable or runaway
%   runs get a large penalty. The weights are fixed BEFORE tuning.
%
%   16 Oct 2026: a simulation that fails for a computer reason (disk or
%   database full, out of memory, logging could not be written) is no longer
%   scored as J = 100. It now stops the run with an error, so bad gains and
%   a broken computer can never be mixed up in the GA/PSO results.
w1 = 0.01;  w2 = 0.005;
if nargin < 3, engine = 'simulink'; end
if ~isempty(x), P = apply_gains(P, x); end
scen = make_scenario('tuning', P);
try
    res = run_sim(P, scen, engine);
    if res.t(end) < scen.Tend - 2*P.ctrl.Ts || any(~isfinite(res.lp(:)))
        J = 100;  M = struct();  return;           % diverged
    end
    M = compute_metrics(res);
    J = M.itae + w1*M.overshoot + w2*M.Te_ripple_rms;
    I = log_index();
    ferr = abs(res.lc(end, I.c.w_ref) - res.lp(end, I.p.w))/P.m.wm_rated;
    if ~isfinite(J) || ferr > 0.1, J = 10 + ferr; end
catch err
    infra = {'Cannot create variable', 'database', 'disk', ...
             'out of memory', 'Out of memory', 'No space'};
    if contains(err.message, infra, 'IgnoreCase', true)
        error('tuning_cost:infrastructure', ...
              'Simulation failed for a computer reason, not the gains: %s', err.message);
    end
    warning('tuning_cost:failed', '%s', err.message);
    J = 100;  M = struct();
end
end
