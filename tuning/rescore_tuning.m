function T = rescore_tuning(engine)
%RESCORE_TUNING  Re-evaluate the tuning cost J of the stored GA / PSO / ZN results
%   with the corrected rms-ripple integration in plant_step.m (17 Oct 2026),
%   without re-running the optimisers.
%
%   T = rescore_tuning            (Simulink engine)   or   rescore_tuning('matlab')
%
%   1. J of the best gains of every GA/PSO run (results/tuning_runs.mat) and of the
%      Ziegler-Nichols gains, old value next to the corrected value.
%   2. Optimum check: the overall best gains of each drive are moved by +/-0.05 decade
%      in every gain (inside the bounds). If no neighbour has a lower corrected J,
%      the optimum has not moved and the tuned gains stand.
%   Writes results/tuning_rescored.mat and results/table_tuning_rescored.csv
%   (export_tables adds the corrected J to table_tuning_runs.csv).

if nargin < 1, engine = 'simulink'; end
setup_project();
root = fileparts(fileparts(mfilename('fullpath')));
rd = fullfile(root, 'results');
S = load(fullfile(rd, 'tuned_params.mat'), 'Pi_zn', 'Pd_zn');
U = load(fullfile(rd, 'tuning_runs.mat'), 'tun');
tun = U.tun;
n = numel(tun);
drive = cell(n + 2, 1);  method = drive;  seed = NaN(n + 2, 1);  J_old = seed;  J_new = seed;
for k = 1:n
    o = tun(k).out;
    P0 = base(S, tun(k).drive);
    drive{k} = tun(k).drive;  method{k} = upper(tun(k).method);  seed(k) = o.seed;  J_old(k) = o.J;
    J_new(k) = tuning_cost(o.x(:)', P0, engine);
    fprintf('%-4s %-3s seed %d: J %.5f -> %.5f\n', drive{k}, method{k}, seed(k), J_old(k), J_new(k));
end
zn = {'IFOC', S.Pi_zn; 'DTC', S.Pd_zn};
for d = 1:2
    k = n + d;
    drive{k} = zn{d,1};  method{k} = 'ZN';
    J_new(k) = tuning_cost([], zn{d,2}, engine);
    fprintf('%-4s ZN: J = %.5f\n', drive{k}, J_new(k));
end
T = table(drive, method, seed, J_old, J_new);
% ---- optimum check around the best gains of each drive
check = struct('drive', {}, 'x_best', {}, 'J_best', {}, 'x_nb', {}, 'J_nb', {}, 'moved', {});
for dname = {'IFOC', 'DTC'}
    idx = find(strcmp(drive(1:n), dname{1}));
    [~, j] = min(J_new(idx));  kb = idx(j);
    P0 = base(S, dname{1});
    [lb, ub] = tuning_bounds(P0);
    xb = tun(kb).out.x(:)';  Jb = J_new(kb);
    X = [];  Jn = [];
    for i = 1:numel(xb)
        for st = [-0.05 0.05]
            x = xb;  x(i) = min(max(x(i) + st, lb(i)), ub(i));
            if abs(x(i) - xb(i)) < 1e-12, continue; end
            X(end+1, :) = x; %#ok<AGROW>
            Jn(end+1, 1) = tuning_cost(x, P0, engine); %#ok<AGROW>
        end
    end
    moved = any(Jn < Jb - 1e-9);
    check(end+1) = struct('drive', dname{1}, 'x_best', xb, 'J_best', Jb, 'x_nb', X, 'J_nb', Jn, 'moved', moved); %#ok<AGROW>
    fprintf('%s best (%s %s seed %d): J = %.5f; lowest neighbour J = %.5f -> optimum %s\n', dname{1}, ...
        method{kb}, mat2str(xb, 4), seed(kb), Jb, min(Jn), ternary(moved, 'MOVED (consider re-tuning)', 'unchanged'));
end
save(fullfile(rd, 'tuning_rescored.mat'), 'T', 'check');
writetable(T, fullfile(rd, 'table_tuning_rescored.csv'));
G = groupsummary(T(~strcmp(T.method, 'ZN'), :), {'drive', 'method'}, {'mean', 'std', 'min'}, {'J_old', 'J_new'});
disp(T);  disp(G);
end

function P = base(S, d)
if strcmp(d, 'IFOC'), P = S.Pi_zn; else, P = S.Pd_zn; end
end

function s = ternary(c, a, b)
if c, s = a; else, s = b; end
end
