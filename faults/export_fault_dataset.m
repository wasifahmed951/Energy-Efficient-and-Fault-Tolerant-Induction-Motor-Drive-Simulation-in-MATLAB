function DS = export_fault_dataset(Pifoc, Pdtc, opts)
%EXPORT_FAULT_DATASET  Labelled fault windows for the Python classifiers
%   (Mamba2, XGBoost, KAN, 1-D CNN; proposal 5.5).
%
%   Grid (default 576 runs): 2 drives x 4 classes (healthy, speed sensor,
%   current sensor, open phase) x 6 speeds x 4 loads x 3 onset times.
%   Detection and reconfiguration are switched OFF so the windows show the
%   raw fault signature the classifier must learn.
%
%   Signals the controller can see, resampled to 10 kHz:
%     ia_m, ib_m, ic_m (measured), w_m (measured), w_ref, Te_ref
%   Windows: 20 ms = 200 samples. Per run: 2 windows before the onset
%   (label 0) and 3 after it (label = class).
%
%   Saves dataset/fault_windows.mat with
%     X    [n x 200 x 6]  single
%     y    [n x 1]        0 healthy, 1 speed sensor, 2 current sensor, 3 open phase
%     meta [n x 6]        drive, speed [rad/s], load [N m], onset [s], window start [s], run id
%   Python: scipy.io.loadmat('fault_windows.mat')
if nargin < 3, opts = struct(); end
engine = getf(opts, 'engine', 'simulink');
par = getf(opts, 'parallel', true);
wr = Pifoc.m.wm_rated;  Tr = Pifoc.m.T_rated;
speeds = getf(opts, 'speeds', linspace(0.2, 0.95, 6)*wr);
loads  = getf(opts, 'loads', [0.1 0.3 0.5 0.7]*Tr);
onsets = getf(opts, 'onsets', [0.5 0.6 0.7]);
classes = {'none', 'speed', 'current', 'open'};
drives = {Pifoc, Pdtc};
Pl = {};  key = [];
for d = 1:2
    for c = 1:4
        for s = 1:numel(speeds)
            for l = 1:numel(loads)
                for o = 1:numel(onsets)
                    Pi = drives{d};  Pi.ctrl.fd_on = 0;
                    Pl{end+1} = Pi; %#ok<AGROW>
                    key(end+1,:) = [d c-1 speeds(s) loads(l) onsets(o)]; %#ok<AGROW>
                end
            end
        end
    end
end
fprintf('Fault dataset: %d runs\n', numel(Pl));
I = log_index();
L = 200;  nwin = 5;
root = fileparts(fileparts(mfilename('fullpath')));
Sl = @(i) fault_scen(Pl{i}, classes{key(i,2)+1}, key(i,3), key(i,4), key(i,5));
red = @(r, i) run_windows(r, i, key, I, L, nwin);
R = run_batch(Pl, Sl, engine, par, struct('reduce', red, 'chunk', 48, ...
    'ckpt', fullfile(root, 'dataset', 'ckpt_fault_windows.mat')));
X = zeros(numel(R)*nwin, L, 6, 'single');  y = zeros(numel(R)*nwin, 1);  meta = zeros(numel(R)*nwin, 6);
n = 0;
for i = 1:numel(R)
    if isempty(R{i}), continue; end
    m = numel(R{i}.y);
    X(n+1:n+m,:,:) = R{i}.X;  y(n+1:n+m) = R{i}.y;  meta(n+1:n+m,:) = R{i}.meta;
    n = n + m;
end
X = X(1:n,:,:);  y = y(1:n);  meta = meta(1:n,:);
f = fullfile(root, 'dataset', 'fault_windows.mat');
channels = {'ia_m','ib_m','ic_m','w_m','w_ref','Te_ref'};  %#ok<NASGU>
labels = {'healthy','speed_sensor','current_sensor','open_phase'};  %#ok<NASGU>
meta_cols = {'drive','speed','load','onset','window_start','run'};  %#ok<NASGU>
save(f, 'X', 'y', 'meta', 'channels', 'labels', 'meta_cols', '-v7');
fprintf('Wrote %s (%d windows)\n', f, n);
DS = struct('X', X, 'y', y, 'meta', meta);
end

function sc = fault_scen(Pi, cls, speed, load, onset)
sc = make_scenario('fault', Pi, struct('fault', cls, 't_fault', onset, 'T', load, 'Tend', onset + 0.08));
% replace the speed ramp: reach the test speed by 0.3 s
tt = sc.t;  sc.ref(:,1) = min(max((tt - 0.1)/0.2, 0), 1)*speed;
sc.sl_ref = [tt, sc.ref];
end

function W = run_windows(r, i, key, I, L, nwin)
% cut the labelled windows out of one run (keeps only 5 x 200 x 6 values)
sc = r.scen;
Ts = r.t(2) - r.t(1);  dec = max(1, round(1e-4/Ts));
refm = interp1(sc.t, sc.ref, r.t, 'previous', 'extrap');
ia_m = r.lp(:, I.p.ia).*(1 - refm(:,4));        % sensor faults applied as in plant_step
w_m = r.lp(:, I.p.w).*(1 - refm(:,3));
S = [ia_m, r.lp(:, I.p.ib), r.lp(:, I.p.ic), w_m, r.lc(:, I.c.w_ref), r.lc(:, I.c.Te_ref)];
S = S(dec:dec:end, :);  ts = r.t(dec:dec:end);
on = key(i,5);
starts = [on - 0.045, on - 0.022, on + 0.002, on + 0.025, on + 0.048];
X = zeros(nwin, L, 6, 'single');  y = zeros(nwin, 1);  meta = zeros(nwin, 6);
n = 0;
for w = 1:nwin
    k0 = find(ts >= starts(w), 1);
    if isempty(k0) || k0 + L - 1 > size(S,1), continue; end
    n = n + 1;
    X(n,:,:) = single(S(k0:k0+L-1, :));
    if w <= 2, y(n) = 0; else, y(n) = key(i,2); end
    meta(n,:) = [key(i,1) key(i,3) key(i,4) on starts(w) i];
end
W = struct('X', X(1:n,:,:), 'y', y(1:n), 'meta', meta(1:n,:));
end

function v = getf(s, f, d)
if isfield(s, f), v = s.(f); else, v = d; end
end
