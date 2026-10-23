function T = run_efficiency_compare(P, opts)
%RUN_EFFICIENCY_COMPARE  Rated flux vs loss-minimising flux at light load (RQ3).
%
%   Operating points: speeds 0.3/0.6/0.9 rated x loads 25%/50% rated.
%   Flux strategies: 1 = rated, 3 = look-up table (P.lut), 4 = MLP (if
%   P.mlp has been filled from Python). Writes results/efficiency_<tag>.csv.
if nargin < 2, opts = struct(); end
engine = getf(opts, 'engine', 'simulink');
par = getf(opts, 'parallel', true);
wr = P.m.wm_rated;  Tr = P.m.T_rated;
wv = getf(opts, 'w', [0.3 0.6 0.9]*wr);
Tv = getf(opts, 'T', [0.25 0.5]*Tr);
modes = [1 3];
if any(P.mlp.W1(:) ~= 0), modes = [modes 4]; end
tag = 'IFOC';  if P.ctrl.type == 2, tag = 'DTC'; end
Pl = {};  Sl = {};  key = [];
for a = 1:numel(wv)
    for b = 1:numel(Tv)
        for mm = modes
            Pi = P;  Pi.ctrl.flux_mode = mm;
            Pl{end+1} = Pi; %#ok<AGROW>
            Sl{end+1} = make_scenario('steady', Pi, struct('w', wv(a), 'T', Tv(b), 'Tend', 1.2)); %#ok<AGROW>
            key(end+1,:) = [wv(a) Tv(b) mm]; %#ok<AGROW>
        end
    end
end
cols = {'w','T','flux_mode','psi_ref','P_in','eta_drive','eta_motor','P_fe','P_cus','P_cur', ...
        'Te_ripple_rms','thd50','saving_pct'};
I = log_index();
red = @(r, i) eff_row(r, i, key, I);
R = run_batch(Pl, Sl, engine, par, struct('reduce', red));
D = NaN(numel(R), numel(cols));
for i = 1:numel(R)
    D(i,1:3) = key(i,:);
    if ~isempty(R{i}), D(i,1:12) = R{i}; end
end
for i = 1:size(D,1)
    r = D(:,1) == D(i,1) & D(:,2) == D(i,2) & D(:,3) == 1;
    D(i,13) = (D(r,5) - D(i,5))/D(r,5)*100;
end
root = fileparts(fileparts(mfilename('fullpath')));
f = fullfile(root, 'results', ['efficiency_' tag '.csv']);
write_csv(f, cols, D);
fprintf('Wrote %s\n', f);
T = struct('data', D, 'cols', {cols});
end

function row = eff_row(r, i, key, I)
M = compute_metrics(r);
ix = r.t >= r.scen.win.ss(1);
row = [key(i,:) mean(r.lc(ix, I.c.psi_ref)) M.P_in M.eta_drive M.eta_motor ...
       M.P_fe M.P_cus M.P_cur M.Te_ripple_rms M.thd50];
end

function v = getf(s, f, d)
if isfield(s, f), v = s.(f); else, v = d; end
end
