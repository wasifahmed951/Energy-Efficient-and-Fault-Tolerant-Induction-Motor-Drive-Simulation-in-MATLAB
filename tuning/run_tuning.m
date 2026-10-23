function out = run_tuning(P, method, opts)
%RUN_TUNING  Tune the drive PI gains with GA or PSO under an equal budget.
%
%   out = run_tuning(P, 'ga',  opts)
%   out = run_tuning(P, 'pso', opts)
%
%   opts.budget   objective evaluations (default 600 = 20 x 30)
%   opts.pop      population / swarm size (default 20)
%   opts.seed     random seed (default 1)
%   opts.engine   'simulink' (default) or 'matlab'
%   opts.parallel evaluate the population in parallel (default false)
%   opts.x0       optional starting point(s) in log10 units, e.g. the ZN gains
%
%   Uses MATLAB's ga / particleswarm (Global Optimization Toolbox) when
%   available, otherwise the small built-in ga_basic / pso_basic.
%   out.hist = [evaluations, best J so far] for convergence plots.

if nargin < 3, opts = struct(); end
budget = getf(opts, 'budget', 600);  pop = getf(opts, 'pop', 20);
seed = getf(opts, 'seed', 1);  engine = getf(opts, 'engine', 'simulink');
par = getf(opts, 'parallel', false);  x0 = getf(opts, 'x0', []);
[lb, ub, names] = tuning_bounds(P);
nv = numel(lb);
fun = @(x) tuning_cost(x(:)', P, engine);
if par && strcmpi(engine, 'simulink') && has_toolbox('pct')
    pool = gcp();                                  % one cache folder per worker
    wait(parfevalOnAll(pool, @worker_setup, 0));
end
rng(seed);
hist = zeros(0, 2);
tic;
if has_toolbox('gads')
    if strcmpi(method, 'ga')
        o = optimoptions('ga', 'PopulationSize', pop, 'MaxGenerations', ceil(budget/pop), ...
            'MaxStallGenerations', 1e6, 'FunctionTolerance', 1e-12, 'Display', 'iter', ...
            'UseParallel', par, 'OutputFcn', @ga_out);
        if ~isempty(x0), o.InitialPopulationMatrix = x0; end
        [xb, Jb] = ga(fun, nv, [], [], [], [], lb, ub, [], o);
    else
        o = optimoptions('particleswarm', 'SwarmSize', pop, 'MaxIterations', ceil(budget/pop), ...
            'MaxStallIterations', 1e6, 'FunctionTolerance', 1e-12, 'Display', 'iter', ...
            'UseParallel', par, 'OutputFcn', @pso_out);
        if ~isempty(x0), o.InitialSwarmMatrix = x0; end
        [xb, Jb] = particleswarm(fun, nv, lb, ub, o);
    end
else
    if strcmpi(method, 'ga')
        [xb, Jb, hist] = ga_basic(fun, lb, ub, pop, budget, seed, x0);
    else
        [xb, Jb, hist] = pso_basic(fun, lb, ub, pop, budget, seed, x0);
    end
end
out.method = method;  out.seed = seed;  out.budget = budget;
out.x = xb;  out.J = Jb;  out.names = names;  out.gains = 10.^xb;
out.hist = hist;  out.time_s = toc;
fprintf('%s seed %d: J = %.4f, gains = %s\n', upper(method), seed, Jb, mat2str(out.gains, 4));

    function [state, options, changed] = ga_out(options, state, flag) %#ok<INUSD>
        changed = false;
        if ~isempty(state.Best)
            hist(end+1, :) = [state.FunEval, min(state.Best)];
        end
        if state.FunEval >= budget, state.StopFlag = 'y'; end
    end
    function stop = pso_out(ov, state) %#ok<INUSD>
        hist(end+1, :) = [ov.funccount, ov.bestfval];
        stop = ov.funccount >= budget;
    end
end

function v = getf(s, f, d)
if isfield(s, f), v = s.(f); else, v = d; end
end
