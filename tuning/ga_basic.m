function [xb, Jb, hist] = ga_basic(fun, lb, ub, pop, budget, seed, x0)
%GA_BASIC  Real-coded GA (fallback when ga is unavailable):
%   tournament selection, blend crossover (BLX-0.5), Gaussian mutation, 2 elites.
rng(seed);
nv = numel(lb);  span = ub - lb;
X = lb + rand(pop, nv).*span;
if ~isempty(x0), X(1:size(x0,1), :) = x0; end
J = zeros(pop, 1);
for i = 1:pop, J(i) = fun(X(i,:)); end
n = pop;
[Jb, ib] = min(J);  xb = X(ib,:);
hist = [n, Jb];
ne = 2;  pm = 0.2;  sm = 0.1;
while n < budget
    [~, ord] = sort(J);
    Xn = X(ord(1:ne), :);  Jn = J(ord(1:ne));
    while size(Xn,1) < pop && n < budget
        p1 = tourn(J);  p2 = tourn(J);
        a = -0.5 + 2*rand(1, nv);
        c = X(p1,:) + a.*(X(p2,:) - X(p1,:));
        mut = rand(1, nv) < pm;
        c(mut) = c(mut) + sm*span(mut).*randn(1, sum(mut));
        c = min(max(c, lb), ub);
        Jc = fun(c);  n = n + 1;
        Xn(end+1,:) = c; Jn(end+1,1) = Jc; %#ok<AGROW>
        if Jc < Jb, Jb = Jc; xb = c; end
    end
    X = Xn;  J = Jn;
    hist(end+1, :) = [n, Jb]; %#ok<AGROW>
end
end

function i = tourn(J)
c = randi(numel(J), 2, 1);
if J(c(1)) <= J(c(2)), i = c(1); else, i = c(2); end
end
