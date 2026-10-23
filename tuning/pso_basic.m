function [xb, Jb, hist] = pso_basic(fun, lb, ub, pop, budget, seed, x0)
%PSO_BASIC  Plain global-best PSO (fallback when particleswarm is unavailable).
%   Inertia 0.729, c1 = c2 = 1.49445 (Clerc constriction values).
rng(seed);
nv = numel(lb);  span = ub - lb;
X = lb + rand(pop, nv).*span;
if ~isempty(x0), X(1:size(x0,1), :) = x0; end
V = 0.1*(rand(pop, nv) - 0.5).*span;
J = zeros(pop, 1);
for i = 1:pop, J(i) = fun(X(i,:)); end
n = pop;
pb = X;  pJ = J;
[Jb, ib] = min(J);  xb = X(ib,:);
hist = [n, Jb];
w = 0.729;  c1 = 1.49445;  c2 = 1.49445;
while n < budget
    for i = 1:pop
        if n >= budget, break; end
        V(i,:) = w*V(i,:) + c1*rand(1,nv).*(pb(i,:) - X(i,:)) + c2*rand(1,nv).*(xb - X(i,:));
        X(i,:) = min(max(X(i,:) + V(i,:), lb), ub);
        J(i) = fun(X(i,:));  n = n + 1;
        if J(i) < pJ(i), pJ(i) = J(i); pb(i,:) = X(i,:); end
        if J(i) < Jb, Jb = J(i); xb = X(i,:); end
    end
    hist(end+1, :) = [n, Jb]; %#ok<AGROW>
end
end
