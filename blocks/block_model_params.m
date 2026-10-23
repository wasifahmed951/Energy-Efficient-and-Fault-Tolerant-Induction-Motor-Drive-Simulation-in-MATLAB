%BLOCK_MODEL_PARAMS  Parameters for the block-diagram models
%   IFOC_BLOCKS, DTC_BLOCKS, AGV_IFOC_BLOCKS and AGV_DTC_BLOCKS.
%
%   This script runs automatically before every simulation (model InitFcn),
%   so EDIT THE VALUES HERE, not in the MATLAB workspace.
%
%   The block models use EXACTLY the parameter set of the thesis scripts:
%   the struct P is loaded from results/tuned_params.mat (Pi_best / Pd_best,
%   the same struct main_run_all uses), and every value below is read from P.
%   To try a different value, change P in the "YOUR CHANGES" section; do not
%   type numbers into the flattened variables further down.
%
%   The model sets bm_drive ('IFOC' or 'DTC') and bm_model (its name) before
%   calling this script.

if ~exist('bm_drive', 'var'), bm_drive = 'IFOC'; end
if ~exist('bm_model', 'var'), bm_model = ''; end
bm_here = fileparts(mfilename('fullpath'));
bm_root = fileparts(bm_here);
addpath(bm_root);
setup_project();

%% ---------------- Thesis parameter set ----------------
bm_tp = fullfile(bm_root, 'results', 'tuned_params.mat');
if exist(bm_tp, 'file')
    bm_S = load(bm_tp, 'Pi_best', 'Pd_best');
    if strcmpi(bm_drive, 'IFOC'), P = bm_S.Pi_best; else, P = bm_S.Pd_best; end
else
    warning('block_model_params:untuned', 'results/tuned_params.mat not found: using untuned default_params');
    P = default_params(bm_drive);
end

%% ================= YOUR CHANGES (optional) =================
% Examples:
%   P.ctrl.Kp_w = 8;            % speed PI gain
%   P.agv.fault_time = 30;      % AGV fault time [s]
%   P.ctrl.remedy_open = 0;     % IFOC without the neutral remedy
% ============================================================

%% ---------------- Test cycle when "Reference source" = 2 (thesis scenario) ----------------
% name: 'step_rated' | 'load_step' | 'reversal' | 'tuning' | 'steady' | 'fault'
% (the same make_scenario cycles the thesis scripts run)
scenario       = 'fault';
scenario_opt   = struct('fault', 'none');     % 'fault': 'none' | 'speed' | 'current' | 'open'
                                              % 'steady': struct('w', 90, 'T', 6.7, 'Tend', 1.0)
%% ---------------- Manual test cycle when "Reference source" = 1 ----------------
t_fault  = 0.8;      % time an enabled fault starts [s]
t_manual = 1.3;      % stop time of the manual test cycle [s]

%% ---------------- Solver ----------------
h_step     = 5e-6;   % drive models: fixed step [s] (Ts must be a multiple of it); 1e-6 = closest to the thesis engine
h_step_agv = 25e-6;  % AGV models: fixed step [s]

% compare_block_vs_script sets these in the base workspace to run a given test cycle
if exist('bm_override', 'var') && isstruct(bm_override)
    if isfield(bm_override, 'scenario'), scenario = bm_override.scenario; end
    if isfield(bm_override, 'scenario_opt'), scenario_opt = bm_override.scenario_opt; end
    if isfield(bm_override, 'h_step'), h_step = bm_override.h_step; end
    if isfield(bm_override, 'P'), P = bm_override.P; end
end

%% ======== below: values taken from P (do not edit) ========
bm_scen = make_scenario(scenario, P, scenario_opt);
scen_ref = bm_scen.sl_ref;                          % [t, w_ref, T_load, f_speed, f_cur, f_open, J_extra]
bm_src = 1;
try
    if ~isempty(bm_model), bm_src = str2double(get_param([bm_model '/Reference source'], 'Value')); end
catch
end
if bm_src == 2, t_stop = bm_scen.Tend; else, t_stop = t_manual; end

% ---- motor ----
m = P.m;
Rs = m.Rs;  Rr = m.Rr;  Lls = m.Lls;  Llr = m.Llr;
Lm_rated = m.Lm_rated;  Lm0 = m.Lm0;
sat_on = m.sat_on;  sat_a = m.sat_a;  sat_b = m.sat_b;  psi_k = m.psi_k;
pp = m.p;  Jm = m.J;  Bm = m.B;
psi_r_rated = m.psi_r_rated;  wm_rated = m.wm_rated;  T_rated = m.T_rated;  I_max = m.I_max;
kh = m.kh;  ke = m.ke;
% ---- inverter ----
Vdc = P.inv.Vdc;  fsw = P.inv.fsw;
Vce0 = P.inv.Vce0;  rce = P.inv.rce;  Esw = P.inv.Esw;  Vref_sw = P.inv.Vref;  Iref_sw = P.inv.Iref;
% ---- controller ----
c = P.ctrl;
drive_type = c.type;  Ts = c.Ts;
Kp_w = c.Kp_w;  Ki_w = c.Ki_w;  Te_max = c.Te_max;
relay_T = c.relay_T;  relay_h = c.relay_h;  relay_bias = c.relay_bias;
Kp_i = c.Kp_i;  Ki_i = c.Ki_i;
ctrl_sat = double(c.ctrl_sat_aware > 0 && sat_on > 0);   % controller uses Lm(psi_ref)
h_psi = c.h_psi;  h_T = c.h_T;  wc_cm = c.wc_cm;
psi_fixed = c.psi_fixed;  psi_min = c.psi_min;  psi_rate = c.psi_rate;
trans_dw = c.trans_dw;  trans_dT = c.trans_dT;  trans_hold = c.trans_hold;
Ko = c.Ko;  mras_Kp = c.mras_Kp;  mras_Ki = c.mras_Ki;  mras_wc = c.mras_wc;
fd_on = c.fd_on;  tdo_w_thr = c.tdo_w_thr;  tdo_i_thr = c.tdo_i_thr;
res_w_thr = c.res_w_thr;  res_i_thr = c.res_i_thr;  res_N = c.res_N;  open_N_time = c.open_N_time;
remedy_open = c.remedy_open;  derate_open = c.derate_open;  derate_sensor = c.derate_sensor;
wmin = 0.1*wm_rated;
flux_mode_default = c.flux_mode;  speed_mode_default = c.speed_mode;
% PWM: Ts is half a carrier period (double update); NT solver steps per Ts
NT = round(Ts/h_step);
NT_agv = max(1, round(Ts/h_step_agv));

% ---- loss-minimising flux tables (rows = speed lut_w [rad/s], columns = torque lut_T [N m]) ----
lut_w = P.lut.w(:)';  lut_T = P.lut.T(:)';
if max(P.lut.psi(:)) - min(P.lut.psi(:)) < 1e-9, bm_P = build_flux_lut(P, 'analytic'); P.lut = bm_P.lut; end
lut_an = P.lut.psi;                                   % analytic loss model (thesis flux_mode 3)
lut_ml = lut_an;                                      % ML table (deployed: KAN for IFOC, MLP for DTC)
if strcmpi(bm_drive, 'IFOC'), bm_f = 'lut_kan_IFOC.csv'; else, bm_f = 'lut_mlp_DTC.csv'; end
bm_f = fullfile(bm_root, 'python_out', bm_f);
if exist(bm_f, 'file'), bm_P = build_flux_lut(P, 'csv', bm_f); lut_ml = bm_P.lut.psi; end
% MLP 2-10-1 (Flux select = 4): python_out/mlp_<drive>.mat if exported, else P.mlp
bm_M = P.mlp;
bm_f = fullfile(bm_root, 'python_out', ['mlp_' upper(bm_drive) '.mat']);
if exist(bm_f, 'file'), bm_L = load(bm_f); bm_M = bm_L.mlp; end
mlp_W1 = double(bm_M.W1);  mlp_b1 = double(bm_M.b1(:));
mlp_W2 = double(bm_M.W2(:)');  mlp_b2 = double(bm_M.b2);
mlp_x_mu = double(bm_M.x_mu(:));  mlp_x_sd = double(bm_M.x_sd(:));
mlp_y_mu = double(bm_M.y_mu);  mlp_y_sd = double(bm_M.y_sd);

% ---- AGV ----
a = P.agv;
m_robot = a.m_robot;  m_payload = a.m_payload;  r_wheel = a.r_wheel;  W_track = a.W_track;
G_gear = a.G;  eta_g = a.eta_g;  Crr = a.Crr;
v_max = a.v_max;  a_max = a.a_max;  a_lat = a.a_lat;  L_look = a.L_look;
dwell_t = a.dwell;  v_limp = a.v_limp;  Lx = a.Lx;  Ly = a.Ly;  t_start = a.t_start;
t_stop_agv = a.Tend;  agv_fault_time = a.fault_time;
agv_fault_type_default = a.fault_type;  agv_fault_side_default = a.fault_side;
sB = Lx + Ly;  sEnd = 2*(Lx + Ly);

% ---- constants used inside Fcn blocks ----
s32 = sqrt(3)/2;  twopi = 2*pi;  pi3 = pi/3;  pi6 = pi/6;
rpm2rad = 2*pi/60;  rad2rpm = 60/(2*pi);

clear bm_S bm_P bm_M bm_L bm_f bm_tp bm_here bm_root bm_src m c a
