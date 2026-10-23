function [M, names] = metrics_table(Mlist, names)
%METRICS_TABLE  Stack a cell array of metric structs into a matrix.
if nargin < 2, names = fieldnames(Mlist{1})'; end
M = NaN(numel(Mlist), numel(names));
for i = 1:numel(Mlist)
    for j = 1:numel(names)
        if isfield(Mlist{i}, names{j}), M(i,j) = Mlist{i}.(names{j}); end
    end
end
end
