function T = run_fault_study(Pifoc, Pdtc, opts)
%RUN_FAULT_STUDY  Sensor and open-phase faults on both drives (RQ4).
%
%   Cases: drive {IFOC, DTC} x fault {speed, current, open} x flux {rated, LUT}
%          plus IFOC open phase with and without the neutral remedy.
%   Fault at 0.8 s, 0.5 rated speed, 50% load. Writes results/fault_study.csv.
if nargin < 3, opts = struct(); end
engine = getf(opts, 'engine', 'simulink');
par = getf(opts, 'parallel', true);
faults = {'speed', 'current', 'open'};
Pl = {};  Sl = {};  key = [];
drives = {Pifoc, Pdtc};
for d = 1:2
    for f = 1:3
        for fm = [1 3]
            remedies = 1;
            if d == 1 && f == 3, remedies = [1 0]; end
            for rm = remedies
                Pi = drives{d};  Pi.ctrl.flux_mode = fm;  Pi.ctrl.remedy_open = rm;
                Pl{end+1} = Pi; %#ok<AGROW>
                Sl{end+1} = make_scenario('fault', Pi, struct('fault', faults{f})); %#ok<AGROW>
                key(end+1,:) = [d f fm rm]; %#ok<AGROW>
            end
        end
    end
end
cols = {'drive(1=IFOC,2=DTC)','fault(1=speed,2=current,3=open)','flux_mode','remedy', ...
        'detected_ok','t_detect_ms','pre_Te_ripple_pp','post_Te_ripple_pp','post_speed_err_pct', ...
        'post_Te_mean','post_I_peak','post_thd50','post_eta_drive'};
I = log_index();
red = @(r, i) fault_row(r, i, key, I);
R = run_batch(Pl, Sl, engine, par, struct('reduce', red));
D = NaN(numel(R), numel(cols));
for i = 1:numel(R)
    D(i,1:4) = key(i,:);
    if ~isempty(R{i}), D(i,:) = R{i}; end
end
root = fileparts(fileparts(mfilename('fullpath')));
f = fullfile(root, 'results', 'fault_study.csv');
write_csv(f, cols, D);
fprintf('Wrote %s\n', f);
T = struct('data', D, 'cols', {cols});
end

function row = fault_row(r, i, key, I)
M = compute_metrics(r);
flag_col = [I.c.f_speed I.c.f_cur I.c.f_open];
flags = max(r.lc(:, flag_col), [], 1);
ok = flags(key(i,2)) > 0.5 && sum(flags) == 1;      % right fault, no false alarm
row = [key(i,:) ok M.t_detect M.Te_ripple_pp M.post_Te_ripple_pp M.post_speed_err_pct ...
       M.post_Te_mean M.post_I_peak M.post_thd50 M.post_eta_drive];
end

function v = getf(s, f, d)
if isfield(s, f), v = s.(f); else, v = d; end
end
