function T = compare_block_vs_script(model, varargin)
%COMPARE_BLOCK_VS_SCRIPT  Run a block model and the thesis script on the same test
%   and print the results side by side.
%
%   Drive models (thesis test cycles from make_scenario):
%     compare_block_vs_script('IFOC')                              rated speed step
%     compare_block_vs_script('IFOC', 'load_step')
%     compare_block_vs_script('DTC', 'fault', struct('fault', 'open'))
%     compare_block_vs_script('IFOC', 'steady', struct('w', 90, 'T', 6.7), 3)   flux_mode 3 (LUT)
%     arguments: model, scenario, scenario options, flux_mode (1 rated, 2 fixed, 3 LUT, 4 MLP),
%                solver step of the block model (default 1e-6 IFOC, 5e-6 DTC)
%     The thesis side is run_sim (IM_DRIVE); both use the same P from results/tuned_params.mat
%     and the metrics come from the same compute_metrics.m.
%
%   AGV models (thesis results from results/agv_all_flux.csv, no rerun needed):
%     compare_block_vs_script('AGV_IFOC', 1, 0)     flux_mode 1, fault type 0 (healthy)
%     compare_block_vs_script('AGV_DTC', 3, 3)      analytic LUT, open phase
%
%   Expected agreement (checked before release, after the 17 Oct fixes in plant_step.m and
%   unpack_sim_output.m): drive metrics within 0.1 % (IFOC THD50 about 0.7 %: its value is only
%   0.07 %, so that is 0.0005 percentage points of numerical noise); detection times identical;
%   AGV mission time, energy and path error within about 0.05 %.

here = fileparts(mfilename('fullpath'));
root = fileparts(here);
addpath(here);  addpath(root);
cd(root);  setup_project();
if strncmpi(model, 'AGV', 3)
    T = compare_agv(here, root, model, varargin{:});
else
    T = compare_drive(here, root, upper(model), varargin{:});
end
end

% =====================================================================
function T = compare_drive(here, root, drive, scenario, opt, flux_mode, h)
if nargin < 4 || isempty(scenario), scenario = 'step_rated'; end
if nargin < 5 || isempty(opt), opt = struct(); end
if nargin < 6 || isempty(flux_mode), flux_mode = 1; end
if nargin < 7 || isempty(h)
    if strcmp(drive, 'IFOC'), h = 1e-6; else, h = 5e-6; end
end
P = load_P(root, drive);
P.ctrl.flux_mode = flux_mode;
if flux_mode == 4
    f = fullfile(root, 'python_out', ['mlp_' drive '.mat']);
    if exist(f, 'file'), L = load(f); P.mlp = L.mlp; end
end
scen = make_scenario(scenario, P, opt);
fprintf('Thesis script (run_sim, IM_DRIVE): %s, %s, flux_mode %d ...\n', drive, scenario, flux_mode);
tic;  res = run_sim(P, scen);  fprintf('  %.0f s\n', toc);
M1 = compute_metrics(res);
% ---- block model on the same cycle
mdl = [drive '_BLOCKS'];
if ~exist(fullfile(here, [mdl '.slx']), 'file'), build_block_models(drive); end
load_system(fullfile(here, [mdl '.slx']));
old = {get_param([mdl '/Reference source'], 'Value'), get_param([mdl '/Flux select'], 'Value'), get_param([mdl '/LUT source'], 'Value')};
set_param([mdl '/Reference source'], 'Value', '2');
set_param([mdl '/Flux select'], 'Value', num2str(flux_mode));
set_param([mdl '/LUT source'], 'Value', '1');           % P.lut, as the script
assignin('base', 'bm_override', struct('scenario', scenario, 'scenario_opt', opt, 'h_step', h, 'P', P));
fprintf('Block model %s (step %g s) ...\n', mdl, h);
try
    tic;  out = sim(mdl);  fprintf('  %.0f s\n', toc);
catch err
    evalin('base', 'clear bm_override');
    restore(mdl, old);
    rethrow(err);
end
evalin('base', 'clear bm_override');
restore(mdl, old);
L = out.get('bm_log');
[t, lp, lc] = block_log_to_thesis(L.time, L.signals.values, P);
r2 = res;  r2.t = t;  r2.lp = lp;  r2.lc = lc;
M2 = compute_metrics(r2);
% ---- table
f = {'rise_time', 's'; 'overshoot', '%'; 'settling_time', 's'; 'itae', ''; 'load_dip_rpm', 'rpm'; 'load_recovery', 's'; ...
     'Te_ripple_pp', '% Tr'; 'Te_ripple_rms', '% Tr'; 'thd50', '%'; 'thd_total', '%'; 'fsw_avg', 'Hz'; ...
     'P_in', 'W'; 'P_fe', 'W'; 'P_sw', 'W'; 'P_out', 'W'; 'eta_drive', ''; 't_detect', 'ms'; ...
     'post_Te_ripple_pp', '% Tr'; 'post_speed_err_pct', '%'; 'post_Te_mean', 'Nm'; 'post_I_peak', 'A'; 'post_eta_drive', ''};
names = {};  a = [];  b = [];  u = {};
for k = 1:size(f, 1)
    if isfield(M1, f{k,1}) && (isfinite(M1.(f{k,1})) || isfinite(M2.(f{k,1})))
        names{end+1, 1} = f{k,1};  u{end+1, 1} = f{k,2}; %#ok<AGROW>
        a(end+1, 1) = M1.(f{k,1});  b(end+1, 1) = M2.(f{k,1}); %#ok<AGROW>
    end
end
d = 100*(b - a)./max(abs(a), 1e-12);
T = table(names, u, a, b, d, 'VariableNames', {'metric', 'unit', 'thesis_script', 'block_model', 'diff_pct'});
fprintf('\n%s, %s, flux_mode %d\n', drive, scenario, flux_mode);
disp(T);
end

function restore(mdl, old)
set_param([mdl '/Reference source'], 'Value', old{1});
set_param([mdl '/Flux select'], 'Value', old{2});
set_param([mdl '/LUT source'], 'Value', old{3});
end

% =====================================================================
function T = compare_agv(here, root, model, flux_mode, fault_type)
if nargin < 4 || isempty(flux_mode), flux_mode = 1; end
if nargin < 5 || isempty(fault_type), fault_type = 0; end
drive = upper(strrep(model, 'AGV_', ''));
mdl = ['AGV_' drive '_BLOCKS'];
P = load_P(root, drive);
% thesis result
fl = {'rated', 'fixed', 'analytic LUT', 'MLP'};  ft = {'healthy', 'speed sensor', 'current sensor', 'open phase'};
thesis = [];
f = fullfile(root, 'results', 'agv_all_flux.csv');
if exist(f, 'file')
    C = readtable(f, 'TextType', 'string');
    k = find(C.drive == drive & C.flux == fl{flux_mode} & C.fault == ft{fault_type + 1}, 1);
    if ~isempty(k), thesis = [C.completed(k), C.time_s(k), C.E_Wh(k), C.e_max_cm(k), C.limp(k), C.stopped(k)]; end
end
if isempty(thesis)
    fprintf('No stored thesis result: running run_agv (AGV_MISSION) ...\n');
    P.ctrl.flux_mode = flux_mode;  P.agv.fault_type = fault_type;
    r = run_agv(P);
    thesis = [r.M.completed, r.M.mission_time, r.M.E_Wh, r.M.e_cross_max_cm, r.M.limp, r.M.stopped];
end
% block model
if ~exist(fullfile(here, [mdl '.slx']), 'file'), build_block_models(['AGV_' drive]); end
load_system(fullfile(here, [mdl '.slx']));
old = {get_param([mdl '/Flux select'], 'Value'), get_param([mdl '/Fault type'], 'Value'), get_param([mdl '/LUT source'], 'Value')};
set_param([mdl '/Flux select'], 'Value', num2str(flux_mode));
set_param([mdl '/Fault type'], 'Value', num2str(fault_type));
set_param([mdl '/LUT source'], 'Value', '1');
fprintf('Block model %s (this takes several minutes) ...\n', mdl);
tic;  out = sim(mdl);  fprintf('  %.0f s\n', toc);
set_param([mdl '/Flux select'], 'Value', old{1});
set_param([mdl '/Fault type'], 'Value', old{2});
set_param([mdl '/LUT source'], 'Value', old{3});
A = out.get('agv_log');  Y = squeeze(A.signals.values);
if size(Y, 1) ~= numel(A.time), Y = Y'; end
% agv_log: 1 x 2 y 3 v 4 v_cmd 5 E_Wh 6 e_cross 7 phase 8 limp 9 stop 10 done 11 s 12 e_max 13 t_mission
kd = find(Y(:, 10) > 0.5, 1);  if isempty(kd), kd = size(Y, 1); end
blk = [any(Y(:, 10) > 0.5), Y(kd, 13), Y(kd, 5), 100*Y(end, 12), any(Y(:, 8) > 0.5), any(Y(:, 9) > 0.5)];
names = {'completed'; 'mission_time_s'; 'E_Wh'; 'e_max_cm'; 'limp'; 'stopped'};
d = 100*(blk(:) - thesis(:))./max(abs(thesis(:)), 1e-12);
T = table(names, thesis(:), blk(:), d, 'VariableNames', {'metric', 'thesis_script', 'block_model', 'diff_pct'});
fprintf('\n%s, flux_mode %d, fault type %d\n', mdl, flux_mode, fault_type);
disp(T);
end

function P = load_P(root, drive)
S = load(fullfile(root, 'results', 'tuned_params.mat'), 'Pi_best', 'Pd_best');
if strcmpi(drive, 'IFOC'), P = S.Pi_best; else, P = S.Pd_best; end
end
