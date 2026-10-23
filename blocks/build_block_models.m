function build_block_models(which)
%BUILD_BLOCK_MODELS  Build the thesis models from standard Simulink blocks.
%
%   build_block_models                 builds all four models in this folder
%   build_block_models('IFOC')         IFOC_BLOCKS      one IFOC drive
%   build_block_models('DTC')          DTC_BLOCKS       one DTC drive
%   build_block_models('AGV_IFOC')     AGV_IFOC_BLOCKS  AGV mission with two IFOC wheel drives
%   build_block_models('AGV_DTC')      AGV_DTC_BLOCKS   AGV mission with two DTC wheel drives
%
%   The block models compute the SAME equations as the thesis code
%   (core/ctrl_step.m, core/plant_step.m, agv/agv_step.m), with the same
%   parameter struct P (block_model_params.m loads results/tuned_params.mat):
%     - Controller: discrete time at Ts (100 us IFOC, 25 us DTC). Every
%       controller variable is a Fcn block "x = f(...)", every controller
%       state a Unit Delay "x(k-1)", in the order of ctrl_step.m.
%     - Inverter: centre-aligned double-update PWM with the thesis carrier
%       phase, conduction and switching losses; DTC applies 0/1 vectors.
%     - Motor: saturated magnetising inductance Lm(|psi_m|), alpha-beta flux
%       states, open-phase modes 1/2/3, core loss from the air-gap flux.
%     - AGV: agv_step.m (kinematics, energy, path progress, supervisor,
%       mission logic, pure pursuit, loads, fault injection) every Ts.
%   The remaining difference to the thesis engine is the numerical
%   integration of the motor (fixed-step RK4 at h_step instead of exact
%   integration between switching instants); compare_block_vs_script.m
%   runs both on the same test cycle and prints the results side by side.
%
%   Drive models, top level:
%     Reference source  1 manual blocks, 2 thesis test cycle (make_scenario,
%                       chosen in block_model_params.m)
%     Speed ref (rpm), Load torque (Nm), Speed-sensor / Current-sensor /
%     Open-phase fault (0 off, 1 on from t_fault)  - when Reference source = 1
%     Flux select       1 rated, 2 fixed, 3 look-up table, 4 MLP  (= P.ctrl.flux_mode)
%     LUT source        1 analytic loss model, 2 ML table (KAN IFOC / MLP DTC)
%     Speed controller  1 PI, 2 Ziegler-Nichols relay (= P.ctrl.speed_mode)
%   AGV models, top level: Flux select, LUT source, Fault type, Fault side.
%
%   Rebuild after editing this file:  bdclose all; clear functions; build_block_models

if nargin < 1, which = {'IFOC', 'DTC', 'AGV_IFOC', 'AGV_DTC'}; end
if ischar(which), which = {which}; end
here = fileparts(mfilename('fullpath'));
addpath(here);
for k = 1:numel(which)
    switch upper(which{k})
        case 'IFOC',     build_drive_model(here, 'IFOC');
        case 'DTC',      build_drive_model(here, 'DTC');
        case 'AGV_IFOC', build_agv_model(here, 'IFOC');
        case 'AGV_DTC',  build_agv_model(here, 'DTC');
        otherwise, error('build_block_models: unknown model %s', which{k});
    end
end
end

%% =====================================================================
%  Stand-alone drive model
%  =====================================================================
function build_drive_model(here, type)
mdl = [type '_BLOCKS'];
evalin('base', sprintf('bm_drive = ''%s''; bm_model = ''''; block_model_params;', type));
if strcmp(type, 'IFOC')
    what = 'IFOC drive (indirect rotor-flux-oriented control, SVPWM 5 kHz, Ts = 100 us)';
else
    what = 'Classical DTC drive (hysteresis + six-sector table, Ts = 25 us)';
end
file = new_model(mdl, here, 'h_step', 't_stop', type, sprintf([ ...
    '%s: the thesis equations (ctrl_step.m, plant_step.m) drawn as blocks, thesis parameters P.\n' ...
    'Reference source: 1 = manual blocks (speed, load, faults), 2 = thesis test cycle set in blocks/block_model_params.m.\n' ...
    'Flux select: 1 rated, 2 fixed, 3 look-up table, 4 MLP.  LUT source: 1 analytic, 2 ML.  Speed controller: 1 PI, 2 ZN relay.\n' ...
    'compare_block_vs_script runs this model and the thesis script on the same test cycle and prints both results.'], what), [30 1060 1000 1140]);

% ---- settings ----
B([mdl '/Reference source'], 'simulink/Sources/Constant', [20 20 70 50], 'Value', '1');
B([mdl '/Flux select'], 'simulink/Sources/Constant', [20 620 70 650], 'Value', 'flux_mode_default');
B([mdl '/LUT source'], 'simulink/Sources/Constant', [20 680 70 710], 'Value', '1');
B([mdl '/Speed controller'], 'simulink/Sources/Constant', [20 740 70 770], 'Value', 'speed_mode_default');
% ---- manual references ----
B([mdl '/Speed ref (rpm)'], 'simulink/Sources/Step', [20 90 70 120], 'Time', '0.2', 'Before', '0', 'After', '1000');
G([mdl '/rpm to rad_s'], [100 90 150 120], 'rpm2rad');
B([mdl '/Load torque (Nm)'], 'simulink/Sources/Step', [20 150 70 180], 'Time', '0.6', 'Before', '0', 'After', '13');
B([mdl '/Extra inertia'], 'simulink/Sources/Constant', [20 210 70 240], 'Value', '0');
B([mdl '/Speed-sensor fault'], 'simulink/Sources/Constant', [20 270 70 300], 'Value', '0');
B([mdl '/Current-sensor fault'], 'simulink/Sources/Constant', [20 330 70 360], 'Value', '0');
B([mdl '/Open-phase fault'], 'simulink/Sources/Constant', [20 390 70 420], 'Value', '0');
fault_onset([mdl '/Fault onset'], [110 260 210 430]);
MUX([mdl '/Manual references'], [260 90 265 430], 6);
B([mdl '/Thesis test cycle'], 'simulink/Sources/From Workspace', [160 460 260 500], ...
    'VariableName', 'scen_ref', 'Interpolate', 'off', 'OutputAfterFinalValue', 'Holding final value', 'SampleTime', 'Ts');
B([mdl '/Reference select'], 'simulink/Signal Routing/Multiport Switch', [320 40 360 500], 'Inputs', '2');
B([mdl '/References'], 'simulink/Signal Routing/Demux', [400 40 405 500], 'Outputs', '6');
W(mdl, 'Speed ref (rpm)', 1, 'rpm to rad_s', 1);
W(mdl, 'Speed-sensor fault', 1, 'Fault onset', 1);
W(mdl, 'Current-sensor fault', 1, 'Fault onset', 2);
W(mdl, 'Open-phase fault', 1, 'Fault onset', 3);
W(mdl, 'rpm to rad_s', 1, 'Manual references', 1);
W(mdl, 'Load torque (Nm)', 1, 'Manual references', 2);
for k = 1:3, W(mdl, 'Fault onset', k, 'Manual references', 2 + k); end
W(mdl, 'Extra inertia', 1, 'Manual references', 6);
W(mdl, 'Reference source', 1, 'Reference select', 1);
W(mdl, 'Manual references', 1, 'Reference select', 2);
W(mdl, 'Thesis test cycle', 1, 'Reference select', 3);
W(mdl, 'Reference select', 1, 'References', 1);
% ---- drive ----
drive_core(mdl, type, 'h_step', 'NT', [480 60], {'References', 1}, {'References', 2}, {'References', 6}, ...
    {'References', 3}, {'References', 4}, {'References', 5}, 'Flux select', 'LUT source', 'Speed controller');
B([mdl '/w_u not used'], 'simulink/Sinks/Terminator', [1150 600 1170 620]);
W(mdl, 'Controller', 5, 'w_u not used', 1);
if strcmp(type, 'IFOC')
    B([mdl '/Te_est not used in IFOC'], 'simulink/Sinks/Terminator', [1150 650 1170 670]);
    W(mdl, 'Controller', 10, 'Te_est not used in IFOC', 1);
end
% ---- scopes ----
x0 = 1500;
G([mdl '/w_ref rpm'], [x0 40 x0+50 70], 'rad2rpm');
G([mdl '/w rpm'], [x0 90 x0+50 120], 'rad2rpm');
W(mdl, 'References', 1, 'w_ref rpm', 1);
W(mdl, 'Inverter and motor', 4, 'w rpm', 1);
SCOPE(mdl, 'Speed (rpm)', [x0+100 50], {{'w_ref rpm', 1}, {'w rpm', 1}});
if strcmp(type, 'IFOC')
    SCOPE(mdl, 'Torque (Nm)', [x0+100 150], {{'Controller', 7}, {'Inverter and motor', 5}, {'References', 2}});
    SCOPE(mdl, 'Rotor flux (Wb)', [x0+100 350], {{'Controller', 8}, {'Controller', 9}, {'Inverter and motor', 6}});
else
    SCOPE(mdl, 'Torque (Nm)', [x0+100 150], {{'Controller', 7}, {'Inverter and motor', 5}, {'References', 2}, {'Controller', 10}});
    SCOPE(mdl, 'Flux (Wb)', [x0+100 350], {{'Controller', 8}, {'Controller', 9}, {'Inverter and motor', 6}});
end
SCOPE(mdl, 'Phase currents (A)', [x0+100 250], {{'Inverter and motor', 1}, {'Inverter and motor', 2}, {'Inverter and motor', 3}});
G([mdl '/w_est rpm'], [x0 460 x0+50 490], 'rad2rpm');
W(mdl, 'Controller', 6, 'w_est rpm', 1);
SCOPE(mdl, 'Speed true vs MRAS (rpm)', [x0+100 450], {{'w rpm', 1}, {'w_est rpm', 1}});
SCOPE(mdl, 'Fault flags', [x0+100 550], {{'Controller', 11}, {'Controller', 12}, {'Controller', 13}, {'Controller', 4}});
SCOPE(mdl, 'Input power (W)', [x0+100 650], {{'Inverter and motor', 7}});
B([mdl '/Energy integrator'], 'simulink/Continuous/Integrator', [x0+100 760 x0+140 790]);
G([mdl '/J to Wh'], [x0+170 760 x0+220 790], '1/3600');
B([mdl '/Energy (Wh)'], 'simulink/Sinks/Display', [x0+250 760 x0+350 790]);
W(mdl, 'Inverter and motor', 7, 'Energy integrator', 1);
W(mdl, 'Energy integrator', 1, 'J to Wh', 1);
W(mdl, 'J to Wh', 1, 'Energy (Wh)', 1);
% energy meters: exact integrals of the powers, torque and current (per-step averages for compare_block_vs_script)
em = [mdl '/Energy meters'];
S(em, [x0-200 850 x0-60 1000], {'Pel', 'Pcond', 'Psw', 'Pfe', 'Pout', 'Pfric', 'Te', 'ia'}, ...
    {'E_el', 'E_cond', 'E_sw', 'E_fe', 'E_out', 'E_fric', 'int_Te', 'int_ia', 'int_Te2', 'int_ia2'}, 'white');
emi = {'Pel', 'Pcond', 'Psw', 'Pfe', 'Pout', 'Pfric', 'Te', 'ia'};  emo = {'E_el', 'E_cond', 'E_sw', 'E_fe', 'E_out', 'E_fric', 'int_Te', 'int_ia'};
for k = 1:8
    B([em '/' emo{k} ' integrator'], 'simulink/Continuous/Integrator', [200 30+60*(k-1) 240 60+60*(k-1)]);
    W(em, emi{k}, 1, [emo{k} ' integrator'], 1);
    W(em, [emo{k} ' integrator'], 1, emo{k}, 1);
end
sq = {'Te', 'int_Te2'; 'ia', 'int_ia2'};
for k = 1:2
    y = 30 + 60*(7 + k);
    F([em '/' sq{k,1} ' squared'], [100 y 160 y+30], 'u*u');
    B([em '/' sq{k,2} ' integrator'], 'simulink/Continuous/Integrator', [200 y 240 y+30]);
    W(em, sq{k,1}, 1, [sq{k,1} ' squared'], 1);
    W(em, [sq{k,1} ' squared'], 1, [sq{k,2} ' integrator'], 1);
    W(em, [sq{k,2} ' integrator'], 1, sq{k,2}, 1);
end
FIN(em, 400);
emsrc = [8 9 10 11 12 13 5 1];
for k = 1:8, W(mdl, 'Inverter and motor', emsrc(k), 'Energy meters', k); end
% log for compare_block_vs_script (every solver step):
%  1 w_ref 2 w 3 Te 4 ia 5 ib 6 ic 7 Pel 8 Pcond 9 Psw 10 Pfe 11 Pout 12 Pfric 13 transitions
%  14 f_speed 15 f_cur 16 f_open 17 neutral 18 w_est 19 Te_ref 20 psi_ref 21 psi_est 22 |psi_r| 23 |psi_m| 24 mode
%  25-32 E_el E_cond E_sw E_fe E_out E_fric int_Te int_ia   33-35 duties da db dc   36-37 int_Te2 int_ia2
MUX([mdl '/Log mux'], [x0+100 850 x0+105 1000], 37);
src = {{'References', 1}, {'Inverter and motor', 4}, {'Inverter and motor', 5}, {'Inverter and motor', 1}, ...
       {'Inverter and motor', 2}, {'Inverter and motor', 3}, {'Inverter and motor', 8}, {'Inverter and motor', 9}, ...
       {'Inverter and motor', 10}, {'Inverter and motor', 11}, {'Inverter and motor', 12}, {'Inverter and motor', 13}, ...
       {'Inverter and motor', 14}, {'Controller', 11}, {'Controller', 12}, {'Controller', 13}, {'Controller', 4}, ...
       {'Controller', 6}, {'Controller', 7}, {'Controller', 8}, {'Controller', 9}, {'Inverter and motor', 6}, ...
       {'Inverter and motor', 15}, {'Inverter and motor', 16}};
for k = 1:8, src{24 + k} = {'Energy meters', k}; end
for k = 1:3, src{32 + k} = {'Controller', k}; end
src{36} = {'Energy meters', 9};  src{37} = {'Energy meters', 10};
for k = 1:37, W2(mdl, src{k}, 'Log mux', k); end
B([mdl '/Log to workspace'], 'simulink/Sinks/To Workspace', [x0+150 910 x0+250 940], ...
    'VariableName', 'bm_log', 'SaveFormat', 'Structure With Time', 'Decimation', '1', 'MaxDataPoints', 'inf');
W(mdl, 'Log mux', 1, 'Log to workspace', 1);
finish_model(mdl, file);
end

function fault_onset(p, pos)
% manual faults: each enabled fault becomes active at t_fault
S(p, pos, {'speed_on', 'current_on', 'open_on'}, {'f_speed', 'f_cur', 'f_open'}, 'red');
B([p '/Fault time'], 'simulink/Sources/Step', [120 240 160 270], 'Time', 't_fault', 'Before', '0', 'After', '1');
ins = {'speed_on', 'current_on', 'open_on'};  outs = {'f_speed', 'f_cur', 'f_open'};
for k = 1:3
    y = 25 + 70*(k-1);
    nm = sprintf('Active %d', k);
    B([p '/' nm], 'simulink/Math Operations/Product', [250 y 280 y+30]);
    W(p, ins{k}, 1, nm, 1);
    W(p, 'Fault time', 1, nm, 2);
    W(p, nm, 1, outs{k}, 1);
end
FIN(p, 400);
end

function SCOPE(sys, name, xy, srcs)
n = numel(srcs);
if n > 1
    MUX([sys '/' name ' mux'], [xy(1) xy(2) xy(1)+5 xy(2)+60], n);
    for k = 1:n, W2(sys, srcs{k}, [name ' mux'], k); end
    B([sys '/' name], 'simulink/Sinks/Scope', [xy(1)+40 xy(2)+15 xy(1)+70 xy(2)+45]);
    W(sys, [name ' mux'], 1, name, 1);
else
    B([sys '/' name], 'simulink/Sinks/Scope', [xy(1)+40 xy(2)+15 xy(1)+70 xy(2)+45]);
    W2(sys, srcs{1}, name, 1);
end
end

%% =====================================================================
%  Drive core: Controller + Inverter and motor
%   Controller out: 1 da 2 db 3 dc 4 neutral 5 w_u 6 w_est 7 Te_ref 8 psi_ref 9 psi_est
%                   10 Te_est 11 f_speed 12 f_cur 13 f_open
%   Inverter and motor out: 1 ia 2 ib 3 ic 4 w 5 Te 6 |psi_r| 7 P_in 8 Pel 9 Pcond 10 Psw
%                   11 Pfe 12 Pout 13 Pfric 14 transitions 15 |psi_m| 16 mode
%                   17 ia_m 18 ib_m 19 ic_m 20 w_m
%  =====================================================================
function drive_core(sys, type, hvar, NTvar, xy, wref, TL, Jx, fs, fc, fo, fsel, lsel, smode)
x = xy(1);  y = xy(2);
controller([sys '/Controller'], [x y x+200 y+420], type);
plant([sys '/Inverter and motor'], [x+420 y x+620 y+420], hvar, NTvar);
for k = 1:4, W(sys, 'Inverter and motor', 16 + k, 'Controller', k); end
W2(sys, wref, 'Controller', 5);
W2(sys, {fsel, 1}, 'Controller', 6);
W2(sys, {lsel, 1}, 'Controller', 7);
W2(sys, {smode, 1}, 'Controller', 8);
for k = 1:4, W(sys, 'Controller', k, 'Inverter and motor', k); end
W2(sys, TL, 'Inverter and motor', 5);
W2(sys, Jx, 'Inverter and motor', 6);
W2(sys, fs, 'Inverter and motor', 7);
W2(sys, fc, 'Inverter and motor', 8);
W2(sys, fo, 'Inverter and motor', 9);
end

%% =====================================================================
%  CONTROLLER (core/ctrl_step.m), discrete time, sample time Ts
%  =====================================================================
function controller(p, pos, type)
outs = {'da', 'db', 'dc', 'neutral', 'w_u', 'w_est', 'Te_ref', 'psi_ref', 'psi_est', 'Te_est', 'f_speed', 'f_cur', 'f_open'};
S(p, pos, {'ia_m', 'ib_m', 'ic_m', 'w_m', 'w_ref', 'flux_sel', 'lut_sel', 'speed_mode'}, outs, 'green');
nm = {'ia_m', 'ib_m', 'ic_m', 'w_m', 'w_ref'};
for k = 1:5
    yy = 30 + 70*(k-1);
    B([p '/' nm{k} ' sample'], 'simulink/Discrete/Zero-Order Hold', [120 yy 160 yy+30], 'SampleTime', 'Ts');
    W(p, nm{k}, 1, [nm{k} ' sample'], 1);
end
X = 260;
model_inductances([p '/Model inductances'], [X 30 X+180 230]);
current_reconstruction([p '/Current reconstruction'], [X+300 30 X+480 330]);
fdi([p '/Fault detection and isolation'], [X+300 400 X+480 800]);
mras([p '/MRAS speed estimator'], [X 400 X+180 900]);
speed_loop([p '/Speed controller'], [X+600 30 X+780 230]);
flux_reference([p '/Flux reference'], [X+600 300 X+780 560]);
if strcmp(type, 'IFOC')
    inner = 'IFOC current control and SVPWM';
    ifoc_inner([p '/' inner], [X+900 30 X+1080 560]);
else
    inner = 'DTC flux observer and switching table';
    dtc_inner([p '/' inner], [X+900 30 X+1080 560]);
end
observer([p '/Applied voltage and current observer'], [X+600 640 X+780 900]);
CR = 'Current reconstruction';  FD = 'Fault detection and isolation';  MI = 'Model inductances';
OB = 'Applied voltage and current observer';  MR = 'MRAS speed estimator';  FR = 'Flux reference';  SC = 'Speed controller';
% Model inductances: in psi_ref_p | out Lm Ls Lr sLs tr D
W(p, FR, 3, MI, 1);
% Current reconstruction: in ia_m ib_m ic_m f_cur_p neutral_p obs1_p obs3_p Lm Lr D
%   out i_alpha i_beta ia_hat i_alpha_p i_beta_p
W(p, 'ia_m sample', 1, CR, 1);
W(p, 'ib_m sample', 1, CR, 2);
W(p, 'ic_m sample', 1, CR, 3);
W(p, FD, 8, CR, 4);
W(p, FD, 9, CR, 5);
W(p, OB, 3, CR, 6);
W(p, OB, 4, CR, 7);
W(p, MI, 1, CR, 8);
W(p, MI, 3, CR, 9);
W(p, MI, 6, CR, 10);
% MRAS: in i_alpha i_beta i_alpha_p i_beta_p va_app_p vb_app_p Lm Lr sLs tr w_ref w_m
%   f_speed_p trans_t_p dpsi_p Te_ref_p Te_lim_p | out w_est mras_ok
crp = [1 2 4 5];
for k = 1:4, W(p, CR, crp(k), MR, k); end
W(p, OB, 1, MR, 5);
W(p, OB, 2, MR, 6);
W(p, MI, 1, MR, 7);
W(p, MI, 3, MR, 8);
W(p, MI, 4, MR, 9);
W(p, MI, 5, MR, 10);
W(p, 'w_ref sample', 1, MR, 11);
W(p, 'w_m sample', 1, MR, 12);
W(p, FD, 7, MR, 13);
W(p, FR, 4, MR, 14);
W(p, FR, 5, MR, 15);
W(p, SC, 3, MR, 16);
W(p, FD, 10, MR, 17);
% FDI: in w_m ia_m ib_m ic_m w_ref w_est mras_ok ia_hat
%   out f_speed f_cur f_open neutral Te_lim w_u f_speed_p f_cur_p neutral_p Te_lim_p
W(p, 'w_m sample', 1, FD, 1);
W(p, 'ia_m sample', 1, FD, 2);
W(p, 'ib_m sample', 1, FD, 3);
W(p, 'ic_m sample', 1, FD, 4);
W(p, 'w_ref sample', 1, FD, 5);
W(p, MR, 1, FD, 6);
W(p, MR, 2, FD, 7);
W(p, CR, 3, FD, 8);
% Speed controller: in w_ref w_u Te_lim mode | out Te_ref e_w Te_ref_p
W(p, 'w_ref sample', 1, SC, 1);
W(p, FD, 6, SC, 2);
W(p, FD, 5, SC, 3);
W(p, 'speed_mode', 1, SC, 4);
% Flux reference: in e_w Te_ref w_u flux_sel lut_sel | out psi_ref dpsi_rate psi_ref_p trans_t_p dpsi_rate_p
W(p, SC, 2, FR, 1);
W(p, SC, 1, FR, 2);
W(p, FD, 6, FR, 3);
W(p, 'flux_sel', 1, FR, 4);
W(p, 'lut_sel', 1, FR, 5);
if strcmp(type, 'IFOC')
    % in i_alpha i_beta psi_ref dpsi_rate Te_ref w_u neutral Lm Lr sLs tr | out da db dc psir_est
    W(p, CR, 1, inner, 1);
    W(p, CR, 2, inner, 2);
    W(p, FR, 1, inner, 3);
    W(p, FR, 2, inner, 4);
    W(p, SC, 1, inner, 5);
    W(p, FD, 6, inner, 6);
    W(p, FD, 4, inner, 7);
    W(p, MI, 1, inner, 8);
    W(p, MI, 3, inner, 9);
    W(p, MI, 4, inner, 10);
    W(p, MI, 5, inner, 11);
    B([p '/Te_est not used in IFOC'], 'simulink/Sources/Constant', [X+1150 600 X+1200 630], 'Value', '0');
    W(p, 'Te_est not used in IFOC', 1, 'Te_est', 1);
else
    % in i_alpha i_beta i_alpha_p i_beta_p va_app_p vb_app_p psi_ref Te_ref w_u Lm Ls Lr sLs tr
    % out da db dc psis_mag Te_est
    for k = 1:4, W(p, CR, crp(k), inner, k); end
    W(p, OB, 1, inner, 5);
    W(p, OB, 2, inner, 6);
    W(p, FR, 1, inner, 7);
    W(p, SC, 1, inner, 8);
    W(p, FD, 6, inner, 9);
    for k = 1:5, W(p, MI, k, inner, 9 + k); end
    W(p, inner, 5, 'Te_est', 1);
    B([p '/dpsi_rate not used in DTC'], 'simulink/Sinks/Terminator', [X+800 600 X+820 620]);
    W(p, FR, 2, 'dpsi_rate not used in DTC', 1);
end
% observer: in da db dc ib_m w_u Lm Ls Lr D | out va_app_p vb_app_p obs1_p obs3_p
for k = 1:3, W(p, inner, k, OB, k); end
W(p, 'ib_m sample', 1, OB, 4);
W(p, FD, 6, OB, 5);
W(p, MI, 1, OB, 6);
W(p, MI, 2, OB, 7);
W(p, MI, 3, OB, 8);
W(p, MI, 6, OB, 9);
% controller outputs
for k = 1:3, W(p, inner, k, outs{k}, 1); end
W(p, FD, 4, 'neutral', 1);
W(p, FD, 6, 'w_u', 1);
W(p, MR, 1, 'w_est', 1);
W(p, SC, 1, 'Te_ref', 1);
W(p, FR, 1, 'psi_ref', 1);
W(p, inner, 4, 'psi_est', 1);
W(p, FD, 1, 'f_speed', 1);
W(p, FD, 2, 'f_cur', 1);
W(p, FD, 3, 'f_open', 1);
FIN(p, X + 1250);
end

function model_inductances(p, pos)
% controller model: secant Lm at the flux reference of the last sample (P.ctrl.ctrl_sat_aware)
S(p, pos, {'psi_ref_p'}, {'Lm_c', 'Ls_c', 'Lr_c', 'sLs_c', 'tr_c', 'D_c'}, 'cyan');
FX(p, 'Lm_c', ['Lm_rated+ctrl_sat*(Lm0/(1+sat_a*(' mx('pr', '0.05') '/psi_k)^sat_b)-Lm_rated)'], {'pr', 'psi_ref_p'});
FX(p, 'Ls_c', 'Lls+Lm', {'Lm', 'Lm_c'});
FX(p, 'Lr_c', 'Llr+Lm', {'Lm', 'Lm_c'});
FX(p, 'sLs_c', 'Ls-Lm*Lm/Lr', {'Lm', 'Lm_c', 'Ls', 'Ls_c', 'Lr', 'Lr_c'});
FX(p, 'tr_c', 'Lr/Rr', {'Lr', 'Lr_c'});
FX(p, 'D_c', 'Ls*Lr-Lm*Lm', {'Lm', 'Lm_c', 'Ls', 'Ls_c', 'Lr', 'Lr_c'});
o = {'Lm_c', 'Ls_c', 'Lr_c', 'sLs_c', 'tr_c', 'D_c'};
for k = 1:6, W(p, o{k}, 1, [o{k} ' out'], 1); end
FIN(p);
end

function current_reconstruction(p, pos)
% two-sensor scheme: phase a from its sensor (or the observer after a current-sensor
% fault) and phase b; after the neutral is connected: phases b and c
S(p, pos, {'ia_m', 'ib_m', 'ic_m', 'f_cur_p', 'neutral_p', 'obs1_p', 'obs3_p', 'Lm_c', 'Lr_c', 'D_c'}, ...
    {'i_alpha', 'i_beta', 'ia_hat', 'i_alpha_p', 'i_beta_p'}, 'cyan');
STATES(p, {'i_alpha', '0'; 'i_beta', '0'});
FX(p, 'ia_hat', '(Lr*o1-Lm*o3)/D', {'Lr', 'Lr_c', 'Lm', 'Lm_c', 'o1', 'obs1_p', 'o3', 'obs3_p', 'D', 'D_c'});
FX(p, 'ia_u', ite('fc>0.5', 'ih', 'ia'), {'fc', 'f_cur_p', 'ih', 'ia_hat', 'ia', 'ia_m'});
FXS(p, 'i_alpha', ite('n>0.5', '-(ib+ic)/3', 'iu'), {'n', 'neutral_p', 'ib', 'ib_m', 'ic', 'ic_m', 'iu', 'ia_u'});
FXS(p, 'i_beta', ite('n>0.5', '(ib-ic)/sqrt(3)', '(iu+2*ib)/sqrt(3)'), {'n', 'neutral_p', 'ib', 'ib_m', 'ic', 'ic_m', 'iu', 'ia_u'});
W(p, 'i_alpha', 1, 'i_alpha out', 1);
W(p, 'i_beta', 1, 'i_beta out', 1);
W(p, 'ia_hat', 1, 'ia_hat out', 1);
W(p, 'i_alpha(k-1)', 1, 'i_alpha_p out', 1);
W(p, 'i_beta(k-1)', 1, 'i_beta_p out', 1);
FIN(p);
end

function mras(p, pos)
% MRAS speed estimator with validity logic (ctrl_step.m, "MRAS speed estimator")
S(p, pos, {'i_alpha', 'i_beta', 'i_alpha_p', 'i_beta_p', 'va_app_p', 'vb_app_p', 'Lm_c', 'Lr_c', 'sLs_c', 'tr_c', ...
    'w_ref', 'w_m', 'f_speed_p', 'trans_t_p', 'dpsi_p', 'Te_ref_p', 'Te_lim_p'}, {'w_est', 'mras_ok'}, 'red');
STATES(p, {'psv_alpha', '0'; 'psv_beta', '0'; 'pri_alpha', '0'; 'pri_beta', '0'; 'lpf_alpha', '0'; 'lpf_beta', '0'; ...
    'mr_int', '0'; 'w_est', '0'; 'mras_t', '0'});
% voltage model (low-pass instead of a pure integrator), trapezoidal current
FXS(p, 'psv_alpha', 'ps+Ts*(v-Rs*0.5*(i+ip)-mras_wc*ps)', {'ps', 'psv_alpha(k-1)', 'v', 'va_app_p', 'i', 'i_alpha', 'ip', 'i_alpha_p'});
FXS(p, 'psv_beta', 'ps+Ts*(v-Rs*0.5*(i+ip)-mras_wc*ps)', {'ps', 'psv_beta(k-1)', 'v', 'vb_app_p', 'i', 'i_beta', 'ip', 'i_beta_p'});
FX(p, 'psr_v_alpha', '(Lr/Lm)*(ps-sLs*i)', {'Lr', 'Lr_c', 'Lm', 'Lm_c', 'ps', 'psv_alpha', 'sLs', 'sLs_c', 'i', 'i_alpha'});
FX(p, 'psr_v_beta', '(Lr/Lm)*(ps-sLs*i)', {'Lr', 'Lr_c', 'Lm', 'Lm_c', 'ps', 'psv_beta', 'sLs', 'sLs_c', 'i', 'i_beta'});
% current model at the estimated speed
FXS(p, 'pri_alpha', 'pa+Ts*((Lm/tr)*i-pa/tr-pp*we*pb)', {'pa', 'pri_alpha(k-1)', 'pb', 'pri_beta(k-1)', 'we', 'w_est(k-1)', ...
    'Lm', 'Lm_c', 'tr', 'tr_c', 'i', 'i_alpha'});
FXS(p, 'pri_beta', 'pb+Ts*((Lm/tr)*i-pb/tr+pp*we*pa)', {'pa', 'pri_alpha(k-1)', 'pb', 'pri_beta(k-1)', 'we', 'w_est(k-1)', ...
    'Lm', 'Lm_c', 'tr', 'tr_c', 'i', 'i_beta'});
FXS(p, 'lpf_alpha', 'l+Ts*mras_wc*(r-l)', {'l', 'lpf_alpha(k-1)', 'r', 'pri_alpha'});
FXS(p, 'lpf_beta', 'l+Ts*mras_wc*(r-l)', {'l', 'lpf_beta(k-1)', 'r', 'pri_beta'});
FX(p, 'eps', '(vb*(ra-la)-va*(rb-lb))/(sqrt(va*va+vb*vb)*sqrt((ra-la)*(ra-la)+(rb-lb)*(rb-lb))+1e-3)', ...
    {'va', 'psr_v_alpha', 'vb', 'psr_v_beta', 'ra', 'pri_alpha', 'rb', 'pri_beta', 'la', 'lpf_alpha', 'lb', 'lpf_beta'});
% validity: below 10 % speed re-synchronise to the speed in use; trusted after 0.5 s
FX(p, 'low_speed', 'abs(wr)<wmin', {'wr', 'w_ref'});
FX(p, 'w_sync', ite('fs>0.5', 'wr', 'wm'), {'fs', 'f_speed_p', 'wr', 'w_ref', 'wm', 'w_m'});
FXS(p, 'mr_int', ite('lo', 'pp*ws', 'mi+Ts*mras_Ki*e'), {'lo', 'low_speed', 'ws', 'w_sync', 'mi', 'mr_int(k-1)', 'e', 'eps'});
FXS(p, 'w_est', ite('lo', 'ws', '(mras_Kp*e+mi)/pp'), {'lo', 'low_speed', 'ws', 'w_sync', 'e', 'eps', 'mi', 'mr_int'});
FXS(p, 'mras_t', '(1-lo)*(1-(((tt>0)+(abs(dp)>0.01))>0))*(mt+Ts)', ...
    {'lo', 'low_speed', 'tt', 'trans_t_p', 'dp', 'dpsi_p', 'mt', 'mras_t(k-1)'});
FX(p, 'mras_ok', '(mt>=0.5)*(abs(T)<0.98*Tl)', {'mt', 'mras_t', 'T', 'Te_ref_p', 'Tl', 'Te_lim_p'});
W(p, 'w_est', 1, 'w_est out', 1);
W(p, 'mras_ok', 1, 'mras_ok out', 1);
FIN(p);
end

function fdi(p, pos)
% sensor and open-phase fault detection, isolation and reconfiguration (ctrl_step.m)
S(p, pos, {'w_m', 'ia_m', 'ib_m', 'ic_m', 'w_ref', 'w_est', 'mras_ok', 'ia_hat'}, ...
    {'f_speed', 'f_cur', 'f_open', 'neutral', 'Te_lim', 'w_u', 'f_speed_p', 'f_cur_p', 'neutral_p', 'Te_lim_p'}, 'red');
STATES(p, {'k', '0'; 'hw1', '0'; 'hw2', '0'; 'hw3', '0'; 'hi1', '0'; 'hi2', '0'; 'hi3', '0'; 'cnt_w', '0'; ...
    'f_speed', '0'; 'cnt_i', '0'; 'anom_t', '0'; 'sum_kcl', '0'; 'sum_exp', '0'; 'anom', '0'; 'f_open', '0'; ...
    'f_cur', '0'; 'Te_lim', 'Te_max'; 'neutral', '0'});
FXS(p, 'k', 'kp+1', {'kp', 'k(k-1)'});
FX(p, 'enable', '(fd_on>0.5)*(k*Ts>0.05)', {'k', 'k'});
FX(p, 'd3w', 'w-3*h1+3*h2-h3', {'w', 'w_m', 'h1', 'hw1(k-1)', 'h2', 'hw2(k-1)', 'h3', 'hw3(k-1)'});
% speed sensor
FX(p, 'res_w', 'abs(w-we)', {'w', 'w_m', 'we', 'w_est'});
FX(p, 'w_ok', '(fc<0.5)*(fo<0.5)*(an<0.5)*(abs(wr)>wmin)', {'fc', 'f_cur(k-1)', 'fo', 'f_open(k-1)', 'an', 'anom(k-1)', 'wr', 'w_ref'});
FXS(p, 'cnt_w', ite('en', ite('ok*mo*(r>res_w_thr)', 'c+1', '0'), 'c'), {'en', 'enable', 'ok', 'w_ok', 'mo', 'mras_ok', 'r', 'res_w', 'c', 'cnt_w(k-1)'});
FX(p, 'speed_fault_found', 'en*(fs<0.5)*ok*(((abs(d)>tdo_w_thr)+(c>=res_N))>0)*(r>res_w_thr)', ...
    {'en', 'enable', 'fs', 'f_speed(k-1)', 'ok', 'w_ok', 'd', 'd3w', 'c', 'cnt_w', 'r', 'res_w'});
FXS(p, 'f_speed', 'fs+s', {'fs', 'f_speed(k-1)', 's', 'speed_fault_found'});
FX(p, 'Te_lim after speed test', ite('s', mn('Tl', 'Te_max*derate_sensor'), 'Tl'), {'s', 'speed_fault_found', 'Tl', 'Te_lim(k-1)'});
% phase a: current sensor or open phase
FX(p, 'test_active', 'en*(fc<0.5)*(fo<0.5)*(((fs<0.5)+mo)>0)', {'en', 'enable', 'fc', 'f_cur(k-1)', 'fo', 'f_open(k-1)', 'fs', 'f_speed', 'mo', 'mras_ok'});
FX(p, 'd3i', 'i-3*h1+3*h2-h3', {'i', 'ia_m', 'h1', 'hi1(k-1)', 'h2', 'hi2(k-1)', 'h3', 'hi3(k-1)'});
FX(p, 'res_i', 'abs(i-ih)', {'i', 'ia_m', 'ih', 'ia_hat'});
FXS(p, 'cnt_i', ite('ac', ite('r>res_i_thr', 'c+1', '0'), 'c'), {'ac', 'test_active', 'r', 'res_i', 'c', 'cnt_i(k-1)'});
FX(p, 'anomaly_found', 'ac*(an<0.5)*(((abs(d)>tdo_i_thr)+(c>=res_N))>0)', {'ac', 'test_active', 'an', 'anom(k-1)', 'd', 'd3i', 'c', 'cnt_i'});
FX(p, 'anomaly_open', 'an+f', {'an', 'anom(k-1)', 'f', 'anomaly_found'});
FX(p, 'classify_running', 'ac*(a>0.5)', {'ac', 'test_active', 'a', 'anomaly_open'});
FXS(p, 'anom_t', '(1-f)*at+r*Ts', {'f', 'anomaly_found', 'at', 'anom_t(k-1)', 'r', 'classify_running'});
FXS(p, 'sum_kcl', '(1-f)*s+r*abs(ib+ic)', {'f', 'anomaly_found', 's', 'sum_kcl(k-1)', 'r', 'classify_running', 'ib', 'ib_m', 'ic', 'ic_m'});
FXS(p, 'sum_exp', '(1-f)*s+r*ri', {'f', 'anomaly_found', 's', 'sum_exp(k-1)', 'r', 'classify_running', 'ri', 'res_i'});
FX(p, 'decide', 'r*(at>=open_N_time)', {'r', 'classify_running', 'at', 'anom_t'});
FX(p, 'n_samples', 'floor(at/Ts+0.5)', {'at', 'anom_t'});
FX(p, 'false_alarm', 'd*(se/(n+(n<0.5))<res_i_thr)', {'d', 'decide', 'se', 'sum_exp', 'n', 'n_samples'});
FX(p, 'is_open_phase', 'd*(1-fa)*(sk<0.2*se)', {'d', 'decide', 'fa', 'false_alarm', 'sk', 'sum_kcl', 'se', 'sum_exp'});
FX(p, 'is_current_sensor', 'd*(1-fa)*(1-o)', {'d', 'decide', 'fa', 'false_alarm', 'o', 'is_open_phase'});
FXS(p, 'anom', 'a*(1-d)', {'a', 'anomaly_open', 'd', 'decide'});
FXS(p, 'f_open', 'fo+o', {'fo', 'f_open(k-1)', 'o', 'is_open_phase'});
FXS(p, 'f_cur', 'fc+c', {'fc', 'f_cur(k-1)', 'c', 'is_current_sensor'});
FX(p, 'Te_lim after open test', ite('o', mn('T', 'Te_max*derate_open'), 'T'), {'o', 'is_open_phase', 'T', 'Te_lim after speed test'});
FXS(p, 'Te_lim', ite('c', mn('T', 'Te_max*derate_sensor'), 'T'), {'c', 'is_current_sensor', 'T', 'Te_lim after open test'});
FXS(p, 'neutral', 'n+(1-n)*o*(drive_type<1.5)*(remedy_open>0.5)', {'n', 'neutral(k-1)', 'o', 'is_open_phase'});
% last three readings (third-difference test), always updated
FXS(p, 'hw1', 'w', {'w', 'w_m'});
FXS(p, 'hw2', 'h', {'h', 'hw1(k-1)'});
FXS(p, 'hw3', 'h', {'h', 'hw2(k-1)'});
FXS(p, 'hi1', 'i', {'i', 'ia_m'});
FXS(p, 'hi2', 'h', {'h', 'hi1(k-1)'});
FXS(p, 'hi3', 'h', {'h', 'hi2(k-1)'});
FX(p, 'w_u', ite('fs>0.5', 'we', 'wm'), {'fs', 'f_speed', 'we', 'w_est', 'wm', 'w_m'});
o = {'f_speed', 'f_cur', 'f_open', 'neutral', 'Te_lim', 'w_u'};
for k = 1:6, W(p, o{k}, 1, [o{k} ' out'], 1); end
o = {'f_speed', 'f_cur', 'neutral', 'Te_lim'};
for k = 1:4, W(p, [o{k} '(k-1)'], 1, [o{k} '_p out'], 1); end
FIN(p);
end

function speed_loop(p, pos)
% speed PI with conditional integration (or the ZN relay), torque limit Te_lim
S(p, pos, {'w_ref', 'w_u', 'Te_lim', 'mode'}, {'Te_ref', 'e_w', 'Te_ref_p'}, 'green');
STATES(p, {'relay', '1'; 'Te_ref', '0'; 'int_w', '0'});
FX(p, 'e_w', 'wr-wu', {'wr', 'w_ref', 'wu', 'w_u'});
FXS(p, 'relay', ite('md>1.5', ite('e>relay_h', '1', ite('e<-relay_h', '-1', 'r')), 'r'), {'md', 'mode', 'e', 'e_w', 'r', 'relay(k-1)'});
FX(p, 'Te_u', 'Kp_w*e+iw', {'e', 'e_w', 'iw', 'int_w(k-1)'});
FX(p, 'Te_pi', clampx('Tu', '-Tl', 'Tl'), {'Tu', 'Te_u', 'Tl', 'Te_lim'});
FXS(p, 'Te_ref', ite('md>1.5', 'relay_bias+r*relay_T', 'Tp'), {'md', 'mode', 'r', 'relay', 'Tp', 'Te_pi'});
FXS(p, 'int_w', ite('md>1.5', 'iw', ite('((abs(Tu)<=Tl)+(abs(((e>0)-(e<0))-((Tu>0)-(Tu<0)))>0.5))>0', ...
    clampx('iw+Ts*Ki_w*e', '-Tl', 'Tl'), 'iw')), {'md', 'mode', 'iw', 'int_w(k-1)', 'Tu', 'Te_u', 'Tl', 'Te_lim', 'e', 'e_w'});
W(p, 'Te_ref', 1, 'Te_ref out', 1);
W(p, 'e_w', 1, 'e_w out', 1);
W(p, 'Te_ref(k-1)', 1, 'Te_ref_p out', 1);
FIN(p);
end

function flux_reference(p, pos)
% flux target (rated / fixed / LUT / MLP), rated flux held after a transient, slope limit
S(p, pos, {'e_w', 'Te_ref', 'w_u', 'flux_sel', 'lut_sel'}, {'psi_ref', 'dpsi_rate', 'psi_ref_p', 'trans_t_p', 'dpsi_rate_p'}, 'yellow');
STATES(p, {'trans_t', '0'; 'Te_ref_f', '0'; 'psi_ref', '0'; 'dpsi_rate', '0'});
FXS(p, 'trans_t', mx([ite('((abs(e)>trans_dw)+(abs(T-Tf)>trans_dT))>0', 'trans_hold', 'tt') '-Ts'], '0'), ...
    {'e', 'e_w', 'T', 'Te_ref', 'Tf', 'Te_ref_f(k-1)', 'tt', 'trans_t(k-1)'});
FXS(p, 'Te_ref_f', 'Tf+Ts/0.05*(T-Tf)', {'Tf', 'Te_ref_f(k-1)', 'T', 'Te_ref'});
FX(p, 'abs w_u', 'abs(w)', {'w', 'w_u'});
FX(p, 'abs Te_ref_f', 'abs(T)', {'T', 'Te_ref_f'});
lut2 = 'simulink/Lookup Tables/2-D Lookup Table';
[xx, yy] = LPOS(p);
B([p '/LUT analytic'], lut2, [xx yy xx+80 yy+50], 'Table', 'lut_an', 'BreakpointsForDimension1', 'lut_w', ...
    'BreakpointsForDimension2', 'lut_T', 'ExtrapMethod', 'Clip');
[xx, yy] = LPOS(p);
B([p '/LUT ML'], lut2, [xx yy xx+80 yy+50], 'Table', 'lut_ml', 'BreakpointsForDimension1', 'lut_w', ...
    'BreakpointsForDimension2', 'lut_T', 'ExtrapMethod', 'Clip');
[xx, yy] = LPOS(p);
B([p '/LUT select'], 'simulink/Signal Routing/Multiport Switch', [xx yy xx+40 yy+60], 'Inputs', '2');
[xx, yy] = LPOS(p);
mlp_network([p '/MLP network'], [xx yy xx+100 yy+60]);
nml = {'LUT analytic', 'LUT ML', 'MLP network'};
for k = 1:3
    W(p, 'abs w_u', 1, nml{k}, 1);
    W(p, 'abs Te_ref_f', 1, nml{k}, 2);
end
W(p, 'lut_sel', 1, 'LUT select', 1);
W(p, 'LUT analytic', 1, 'LUT select', 2);
W(p, 'LUT ML', 1, 'LUT select', 3);
FX(p, 'psi_target', ite('tt>0', 'psi_r_rated', clampx(ite('f<1.5', 'psi_r_rated', ite('f<2.5', 'psi_fixed', ite('f<3.5', 'l', 'n'))), ...
    'psi_min', 'psi_r_rated')), {'tt', 'trans_t', 'f', 'flux_sel', 'l', 'LUT select', 'n', 'MLP network'});
FX(p, 'dpsi', clampx('pt-pr', '-psi_rate*Ts', 'psi_rate*Ts'), {'pt', 'psi_target', 'pr', 'psi_ref(k-1)'});
FXS(p, 'psi_ref', 'pr+d', {'pr', 'psi_ref(k-1)', 'd', 'dpsi'});
FXS(p, 'dpsi_rate', 'd/Ts', {'d', 'dpsi'});
W(p, 'psi_ref', 1, 'psi_ref out', 1);
W(p, 'dpsi_rate', 1, 'dpsi_rate out', 1);
W(p, 'psi_ref(k-1)', 1, 'psi_ref_p out', 1);
W(p, 'trans_t(k-1)', 1, 'trans_t_p out', 1);
W(p, 'dpsi_rate(k-1)', 1, 'dpsi_rate_p out', 1);
FIN(p);
end

function mlp_network(p, pos)
% MLP 2-10-1 exported by python/01_flux_models.ipynb: psi = (W2*tanh(W1*xn + b1) + b2)*y_sd + y_mu
S(p, pos, {'w', 'T'}, {'psi'}, 'yellow');
MUX([p '/Input mux'], [100 30 105 110], 2);
B([p '/x_mu'], 'simulink/Sources/Constant', [100 150 160 180], 'Value', 'mlp_x_mu');
SUM([p '/Centre'], [200 60 220 100], '+-');
G([p '/Normalise'], [260 65 330 95], '1./mlp_x_sd');
B([p '/W1'], 'simulink/Math Operations/Gain', [370 65 440 95], 'Gain', 'mlp_W1', 'Multiplication', 'Matrix(K*u)');
B([p '/b1'], 'simulink/Sources/Constant', [370 150 430 180], 'Value', 'mlp_b1');
SUM([p '/Hidden sum'], [480 60 500 100], '++');
B([p '/tanh'], 'simulink/Math Operations/Trigonometric Function', [540 65 590 95], 'Operator', 'tanh');
B([p '/W2'], 'simulink/Math Operations/Gain', [630 65 700 95], 'Gain', 'mlp_W2', 'Multiplication', 'Matrix(K*u)');
B([p '/b2'], 'simulink/Sources/Constant', [630 150 690 180], 'Value', 'mlp_b2');
SUM([p '/Output sum'], [740 60 760 100], '++');
G([p '/y_sd'], [800 65 850 95], 'mlp_y_sd');
B([p '/y_mu'], 'simulink/Sources/Constant', [800 150 860 180], 'Value', 'mlp_y_mu');
SUM([p '/De-normalise'], [900 60 920 100], '++');
W(p, 'w', 1, 'Input mux', 1);  W(p, 'T', 1, 'Input mux', 2);
W(p, 'Input mux', 1, 'Centre', 1);  W(p, 'x_mu', 1, 'Centre', 2);
W(p, 'Centre', 1, 'Normalise', 1);
W(p, 'Normalise', 1, 'W1', 1);
W(p, 'W1', 1, 'Hidden sum', 1);  W(p, 'b1', 1, 'Hidden sum', 2);
W(p, 'Hidden sum', 1, 'tanh', 1);
W(p, 'tanh', 1, 'W2', 1);
W(p, 'W2', 1, 'Output sum', 1);  W(p, 'b2', 1, 'Output sum', 2);
W(p, 'Output sum', 1, 'y_sd', 1);
W(p, 'y_sd', 1, 'De-normalise', 1);  W(p, 'y_mu', 1, 'De-normalise', 2);
W(p, 'De-normalise', 1, 'psi', 1);
FIN(p, 1000);
end

function ifoc_inner(p, pos)
% ifoc_inner in ctrl_step.m: current-model rotor flux, references, PI current
% control with decoupling, voltage limit, inverse Park at mid-period, SVPWM
S(p, pos, {'i_alpha', 'i_beta', 'psi_ref', 'dpsi_rate', 'Te_ref', 'w_u', 'neutral', 'Lm_c', 'Lr_c', 'sLs_c', 'tr_c'}, ...
    {'da', 'db', 'dc', 'psir_est'}, 'cyan');
STATES(p, {'theta', '0'; 'psir_est', '0'; 'int_d', '0'; 'int_q', '0'});
FX(p, 'id', 'cos(th)*ia+sin(th)*ib', {'th', 'theta(k-1)', 'ia', 'i_alpha', 'ib', 'i_beta'});
FX(p, 'iq', '-sin(th)*ia+cos(th)*ib', {'th', 'theta(k-1)', 'ia', 'i_alpha', 'ib', 'i_beta'});
FXS(p, 'psir_est', 'pe+Ts*(Lm*id-pe)/tr', {'pe', 'psir_est(k-1)', 'Lm', 'Lm_c', 'id', 'id', 'tr', 'tr_c'});
FX(p, 'psr', mx('pe', '0.05*psi_r_rated'), {'pe', 'psir_est'});
FX(p, 'id_ref', clampx('pr/Lm+tr*dr/Lm', '0', '0.95*I_max'), {'pr', 'psi_ref', 'Lm', 'Lm_c', 'tr', 'tr_c', 'dr', 'dpsi_rate'});
FX(p, 'iq_max', ['sqrt(' mx('I_max*I_max-d*d', '0') ')'], {'d', 'id_ref'});
FX(p, 'iq_ref', clampx('T/(1.5*pp*(Lm/Lr)*ps)', '-qm', 'qm'), {'T', 'Te_ref', 'Lm', 'Lm_c', 'Lr', 'Lr_c', 'ps', 'psr', 'qm', 'iq_max'});
FX(p, 'we', 'pp*wu+Lm*iq/(tr*ps)', {'wu', 'w_u', 'Lm', 'Lm_c', 'iq', 'iq_ref', 'tr', 'tr_c', 'ps', 'psr'});
FX(p, 'vd_u', 'Kp_i*(dr-d)+ip-we*sLs*q', {'dr', 'id_ref', 'd', 'id', 'ip', 'int_d(k-1)', 'we', 'we', 'sLs', 'sLs_c', 'q', 'iq'});
FX(p, 'vq_u', 'Kp_i*(qr-q)+ip+we*(sLs*d+(Lm/Lr)*pe)', {'qr', 'iq_ref', 'q', 'iq', 'ip', 'int_q(k-1)', 'we', 'we', ...
    'sLs', 'sLs_c', 'd', 'id', 'Lm', 'Lm_c', 'Lr', 'Lr_c', 'pe', 'psir_est'});
FX(p, 'Vmax', ite('n>0.5', 'Vdc/2', 'Vdc/sqrt(3)'), {'n', 'neutral'});
FX(p, 'vmag', 'sqrt(vd*vd+vq*vq)', {'vd', 'vd_u', 'vq', 'vq_u'});
FX(p, 'v_saturated', 'vm>Vx', {'vm', 'vmag', 'Vx', 'Vmax'});
FX(p, 'vd', 'v*((1-s)+s*Vx/(vm+(1-s)))', {'v', 'vd_u', 's', 'v_saturated', 'Vx', 'Vmax', 'vm', 'vmag'});
FX(p, 'vq', 'v*((1-s)+s*Vx/(vm+(1-s)))', {'v', 'vq_u', 's', 'v_saturated', 'Vx', 'Vmax', 'vm', 'vmag'});
FXS(p, 'int_d', 'ip+(1-s)*Ts*Ki_i*(dr-d)', {'ip', 'int_d(k-1)', 's', 'v_saturated', 'dr', 'id_ref', 'd', 'id'});
FXS(p, 'int_q', 'ip+(1-s)*Ts*Ki_i*(qr-q)', {'ip', 'int_q(k-1)', 's', 'v_saturated', 'qr', 'iq_ref', 'q', 'iq'});
FX(p, 'theta_v', 'th+0.5*Ts*we', {'th', 'theta(k-1)', 'we', 'we'});
FX(p, 'v_alpha', 'cos(tv)*vd-sin(tv)*vq', {'tv', 'theta_v', 'vd', 'vd', 'vq', 'vq'});
FX(p, 'v_beta', 'sin(tv)*vd+cos(tv)*vq', {'tv', 'theta_v', 'vd', 'vd', 'vq', 'vq'});
FXS(p, 'theta', 'th+Ts*we-twopi*floor((th+Ts*we)/twopi)', {'th', 'theta(k-1)', 'we', 'we'});
% min-max injection SVPWM; with the neutral connected legs b and c are driven directly
FX(p, 'va_ph', 'va', {'va', 'v_alpha'});
FX(p, 'vb_ph', '-0.5*va+s32*vb', {'va', 'v_alpha', 'vb', 'v_beta'});
FX(p, 'vc_ph', '-0.5*va-s32*vb', {'va', 'v_alpha', 'vb', 'v_beta'});
FX(p, 'v_offset', ['-0.5*(' mx(mx('a', 'b'), 'c') '+' mn(mn('a', 'b'), 'c') ')'], {'a', 'va_ph', 'b', 'vb_ph', 'c', 'vc_ph'});
FX(p, 'da', clampx(['0.5+' ite('n>0.5', '0', 'a+o') '/Vdc'], '0', '1'), {'n', 'neutral', 'a', 'va_ph', 'o', 'v_offset'});
FX(p, 'db', clampx(['0.5+' ite('n>0.5', '(-va+sqrt(3)*vb)/2', 'b+o') '/Vdc'], '0', '1'), ...
    {'n', 'neutral', 'va', 'v_alpha', 'vb', 'v_beta', 'b', 'vb_ph', 'o', 'v_offset'});
FX(p, 'dc', clampx(['0.5+' ite('n>0.5', '(-va-sqrt(3)*vb)/2', 'c+o') '/Vdc'], '0', '1'), ...
    {'n', 'neutral', 'va', 'v_alpha', 'vb', 'v_beta', 'c', 'vc_ph', 'o', 'v_offset'});
W(p, 'da', 1, 'da out', 1);  W(p, 'db', 1, 'db out', 1);  W(p, 'dc', 1, 'dc out', 1);
W(p, 'psir_est', 1, 'psir_est out', 1);
FIN(p);
end

function dtc_inner(p, pos)
% dtc_inner in ctrl_step.m: Jansen-Lorenz stator-flux observer, torque estimate,
% flux and torque hysteresis, sector and six-sector switching table
S(p, pos, {'i_alpha', 'i_beta', 'i_alpha_p', 'i_beta_p', 'va_app_p', 'vb_app_p', 'psi_ref', 'Te_ref', 'w_u', ...
    'Lm_c', 'Ls_c', 'Lr_c', 'sLs_c', 'tr_c'}, {'da', 'db', 'dc', 'psis_mag', 'Te_est'}, 'cyan');
STATES(p, {'psr_cm_alpha', '0'; 'psr_cm_beta', '0'; 'psis_alpha', '0'; 'psis_beta', '0'; 'c_psi', '1'; 'c_T', '0'; ...
    'leg_a', '0'; 'leg_b', '0'; 'leg_c', '0'});
FXS(p, 'psr_cm_alpha', 'pa+Ts*((Lm/tr)*i-pa/tr-pp*wu*pb)', {'pa', 'psr_cm_alpha(k-1)', 'pb', 'psr_cm_beta(k-1)', ...
    'Lm', 'Lm_c', 'tr', 'tr_c', 'i', 'i_alpha', 'wu', 'w_u'});
FXS(p, 'psr_cm_beta', 'pb+Ts*((Lm/tr)*i-pb/tr+pp*wu*pa)', {'pa', 'psr_cm_alpha(k-1)', 'pb', 'psr_cm_beta(k-1)', ...
    'Lm', 'Lm_c', 'tr', 'tr_c', 'i', 'i_beta', 'wu', 'w_u'});
FX(p, 'psis_cm_alpha', '(Lm/Lr)*r+sLs*i', {'Lm', 'Lm_c', 'Lr', 'Lr_c', 'r', 'psr_cm_alpha', 'sLs', 'sLs_c', 'i', 'i_alpha'});
FX(p, 'psis_cm_beta', '(Lm/Lr)*r+sLs*i', {'Lm', 'Lm_c', 'Lr', 'Lr_c', 'r', 'psr_cm_beta', 'sLs', 'sLs_c', 'i', 'i_beta'});
FXS(p, 'psis_alpha', 's+Ts*(v-Rs*0.5*(i+ip)-wc_cm*(s-cm))', {'s', 'psis_alpha(k-1)', 'v', 'va_app_p', 'i', 'i_alpha', ...
    'ip', 'i_alpha_p', 'cm', 'psis_cm_alpha'});
FXS(p, 'psis_beta', 's+Ts*(v-Rs*0.5*(i+ip)-wc_cm*(s-cm))', {'s', 'psis_beta(k-1)', 'v', 'vb_app_p', 'i', 'i_beta', ...
    'ip', 'i_beta_p', 'cm', 'psis_cm_beta'});
FX(p, 'Te_est', '1.5*pp*(sa*ib-sb*ia)', {'sa', 'psis_alpha', 'sb', 'psis_beta', 'ia', 'i_alpha', 'ib', 'i_beta'});
FX(p, 'iq_for_flux_ref', ['T/(1.5*pp*(Lm/Lr)*' mx('pr', '0.05*psi_r_rated') ')'], {'T', 'Te_ref', 'Lm', 'Lm_c', 'Lr', 'Lr_c', 'pr', 'psi_ref'});
FX(p, 'psis_ref', 'sqrt((Ls/Lm*pr)*(Ls/Lm*pr)+(sLs*iq)*(sLs*iq))', {'Ls', 'Ls_c', 'Lm', 'Lm_c', 'pr', 'psi_ref', 'sLs', 'sLs_c', 'iq', 'iq_for_flux_ref'});
FX(p, 'psis_mag', 'sqrt(sa*sa+sb*sb)', {'sa', 'psis_alpha', 'sb', 'psis_beta'});
FXS(p, 'c_psi', ite('m<r-h_psi', '1', ite('m>r+h_psi', '0', 'c')), {'m', 'psis_mag', 'r', 'psis_ref', 'c', 'c_psi(k-1)'});
FX(p, 'e_T', 'T-Te', {'T', 'Te_ref', 'Te', 'Te_est'});
FXS(p, 'c_T', ite('e>h_T', '1', ite('e<-h_T', '-1', ite('((c>0.5)*(e<=0)+(c<-0.5)*(e>=0))>0', '0', 'c'))), {'e', 'e_T', 'c', 'c_T(k-1)'});
FX(p, 'sector', 'floor((atan2(sb,sa)+pi6)/pi3)-6*floor(floor((atan2(sb,sa)+pi6)/pi3)/6)+1', {'sa', 'psis_alpha', 'sb', 'psis_beta'});
FX(p, 'vector before wrap', 's+c*(2-q)', {'s', 'sector', 'c', 'c_T', 'q', 'c_psi'});
FX(p, 'vector', '(kv-1)-6*floor((kv-1)/6)+1', {'kv', 'vector before wrap'});
FX(p, 'magnetise', '(abs(c)<0.5)*(m<0.9*r)', {'c', 'c_T', 'm', 'psis_mag', 'r', 'psis_ref'});
% V = [1 0 0; 1 1 0; 0 1 0; 0 1 1; 0 0 1; 1 0 1]; zero vector with the fewest transitions
legs = {'leg_a', '(abs(%s-1)<0.5)+(abs(%s-2)<0.5)+(abs(%s-6)<0.5)'; ...
        'leg_b', '(abs(%s-2)<0.5)+(abs(%s-3)<0.5)+(abs(%s-4)<0.5)'; ...
        'leg_c', '(abs(%s-4)<0.5)+(abs(%s-5)<0.5)+(abs(%s-6)<0.5)'};
for k = 1:3
    tv = sprintf(legs{k,2}, 'v', 'v', 'v');  ts = sprintf(legs{k,2}, 's', 's', 's');
    FXS(p, legs{k,1}, ite('abs(c)>0.5', tv, ite('mg', ts, '((a+b+cc)>1.5)')), {'c', 'c_T', 'v', 'vector', 's', 'sector', ...
        'mg', 'magnetise', 'a', 'leg_a(k-1)', 'b', 'leg_b(k-1)', 'cc', 'leg_c(k-1)'});
end
W(p, 'leg_a', 1, 'da out', 1);  W(p, 'leg_b', 1, 'db out', 1);  W(p, 'leg_c', 1, 'dc out', 1);
W(p, 'psis_mag', 1, 'psis_mag out', 1);
W(p, 'Te_est', 1, 'Te_est out', 1);
FIN(p);
end

function observer(p, pos)
% applied voltage from the duties (used by the estimators next sample) and the
% phase-a current observer corrected with phase b only (obs_propagate in ctrl_step.m)
S(p, pos, {'da', 'db', 'dc', 'ib_m', 'w_u', 'Lm_c', 'Ls_c', 'Lr_c', 'D_c'}, {'va_app_p', 'vb_app_p', 'obs1_p', 'obs3_p'}, 'cyan');
STATES(p, {'va_app', '0'; 'vb_app', '0'; 'obs1', '0'; 'obs2', '0'; 'obs3', '0'; 'obs4', '0'});
FXS(p, 'va_app', '(2/3)*((a-0.5)*Vdc-0.5*((b-0.5)*Vdc+(c-0.5)*Vdc))', {'a', 'da', 'b', 'db', 'c', 'dc'});
FXS(p, 'vb_app', '((b-0.5)*Vdc-(c-0.5)*Vdc)/sqrt(3)', {'b', 'db', 'c', 'dc'});
oi = {'o1', 'obs1(k-1)', 'o2', 'obs2(k-1)', 'o3', 'obs3(k-1)', 'o4', 'obs4(k-1)', 'Lm', 'Lm_c', 'Ls', 'Ls_c', 'Lr', 'Lr_c', 'D', 'D_c'};
FX(p, 'obs is_alpha', '(Lr*o1-Lm*o3)/D', oi);
FX(p, 'obs is_beta', '(Lr*o2-Lm*o4)/D', oi);
FX(p, 'obs ir_alpha', '(Ls*o3-Lm*o1)/D', oi);
FX(p, 'obs ir_beta', '(Ls*o4-Lm*o2)/D', oi);
FX(p, 'ib error', 'ib-(-0.5*sa+s32*sb)', {'ib', 'ib_m', 'sa', 'obs is_alpha', 'sb', 'obs is_beta'});
FXS(p, 'obs1', 'o1+Ts*(v-Rs*s+Ko*(-0.5)*e)', {'o1', 'obs1(k-1)', 'v', 'va_app', 's', 'obs is_alpha', 'e', 'ib error'});
FXS(p, 'obs2', 'o2+Ts*(v-Rs*s+Ko*s32*e)', {'o2', 'obs2(k-1)', 'v', 'vb_app', 's', 'obs is_beta', 'e', 'ib error'});
FXS(p, 'obs3', 'o3+Ts*(-Rr*r-pp*wu*o4)', {'o3', 'obs3(k-1)', 'o4', 'obs4(k-1)', 'r', 'obs ir_alpha', 'wu', 'w_u'});
FXS(p, 'obs4', 'o4+Ts*(-Rr*r+pp*wu*o3)', {'o3', 'obs3(k-1)', 'o4', 'obs4(k-1)', 'r', 'obs ir_beta', 'wu', 'w_u'});
W(p, 'va_app(k-1)', 1, 'va_app_p out', 1);
W(p, 'vb_app(k-1)', 1, 'vb_app_p out', 1);
W(p, 'obs1(k-1)', 1, 'obs1_p out', 1);
W(p, 'obs3(k-1)', 1, 'obs3_p out', 1);
FIN(p);
end

%% =====================================================================
%  INVERTER AND MOTOR (core/plant_step.m), continuous time
%  =====================================================================
function plant(p, pos, hvar, NTvar)
S(p, pos, {'da', 'db', 'dc', 'neutral', 'TL', 'J_extra', 'f_speed', 'f_cur', 'f_open'}, ...
    {'ia', 'ib', 'ic', 'w', 'Te', 'psir_mag', 'P_in', 'Pel', 'Pcond', 'Psw', 'Pfe', 'Pout', 'Pfric', 'transitions', ...
     'psim_mag', 'mode', 'ia_m', 'ib_m', 'ic_m', 'w_m'}, 'lightBlue');
modulator([p '/PWM modulator'], [200 30 360 330], hvar, NTvar);
induction_motor([p '/Induction motor'], [700 30 880 480]);
inverter_losses([p '/Inverter losses'], [1000 30 1160 330], hvar);
core_loss([p '/Core loss'], [1000 400 1160 520]);
% operating mode: 1 healthy, 2 phase a open (isolated neutral), 3 phase a open + neutral connected.
% Mode changes act one solver step after the command (Memory): the measurement taken at
% that instant still shows the old mode, as in the thesis plant (measurement at the end of
% the previous step); this also breaks the loop neutral -> currents -> controller -> neutral.
B([p '/Neutral contactor'], 'simulink/Discrete/Memory', [420 520 460 550], 'InitialCondition', '0');
B([p '/Phase a opens'], 'simulink/Discrete/Memory', [420 460 460 490], 'InitialCondition', '0');
MUX([p '/Mode mux'], [500 500 505 560], 2);
F([p '/Mode'], [530 515 640 545], '1+u(1)*(1+u(2))');
W(p, 'neutral', 1, 'Neutral contactor', 1);
W(p, 'f_open', 1, 'Phase a opens', 1);
W(p, 'Phase a opens', 1, 'Mode mux', 1);
W(p, 'Neutral contactor', 1, 'Mode mux', 2);
W(p, 'Mode mux', 1, 'Mode', 1);
dn = {'da', 'db', 'dc'};
for k = 1:3, W(p, dn{k}, 1, 'PWM modulator', k); end
% motor: in vpa vpb vpc TL J_extra mode | out ia ib ic w Te psir_mag psim_alpha psim_beta psim_mag
for k = 1:3, W(p, 'PWM modulator', k, 'Induction motor', k); end
W(p, 'TL', 1, 'Induction motor', 4);
W(p, 'J_extra', 1, 'Induction motor', 5);
W(p, 'Mode', 1, 'Induction motor', 6);
% losses: in vpa vpb vpc ia ib ic trans_a trans_b trans_c | out Pel Pcond Psw transitions
for k = 1:3
    W(p, 'PWM modulator', k, 'Inverter losses', k);
    W(p, 'Induction motor', k, 'Inverter losses', 3 + k);
    W(p, 'PWM modulator', 3 + k, 'Inverter losses', 6 + k);
end
W(p, 'Induction motor', 7, 'Core loss', 1);
W(p, 'Induction motor', 8, 'Core loss', 2);
% powers
MUX([p '/Load power mux'], [960 560 965 600], 2);
F([p '/Load power'], [1000 560 1160 590], 'u(1)*u(2)');
W(p, 'TL', 1, 'Load power mux', 1);
W(p, 'Induction motor', 4, 'Load power mux', 2);
W(p, 'Load power mux', 1, 'Load power', 1);
F([p '/Friction power'], [1000 620 1160 650], 'Bm*u*u');
W(p, 'Induction motor', 4, 'Friction power', 1);
SUM([p '/Input power sum'], [1240 100 1260 220], '++++');
W(p, 'Inverter losses', 1, 'Input power sum', 1);
W(p, 'Inverter losses', 2, 'Input power sum', 2);
W(p, 'Inverter losses', 3, 'Input power sum', 3);
W(p, 'Core loss', 1, 'Input power sum', 4);
% sensors (a fault forces the reading to 0). As in the thesis plant, a sensor fault of
% control step k shows in the measurement taken at the end of that step (one Ts later).
B([p '/Speed fault delay'], 'simulink/Discrete/Unit Delay', [900 720 940 750], 'SampleTime', 'Ts', 'InitialCondition', '0');
B([p '/Current fault delay'], 'simulink/Discrete/Unit Delay', [900 790 940 820], 'SampleTime', 'Ts', 'InitialCondition', '0');
W(p, 'f_speed', 1, 'Speed fault delay', 1);
W(p, 'f_cur', 1, 'Current fault delay', 1);
MUX([p '/Speed sensor mux'], [960 700 965 740], 2);
F([p '/Speed sensor'], [1000 700 1160 730], 'u(1)*(1-u(2))');
W(p, 'Induction motor', 4, 'Speed sensor mux', 1);
W(p, 'Speed fault delay', 1, 'Speed sensor mux', 2);
W(p, 'Speed sensor mux', 1, 'Speed sensor', 1);
MUX([p '/Current sensor mux'], [960 770 965 810], 2);
F([p '/Current sensor a'], [1000 770 1160 800], 'u(1)*(1-u(2))');
W(p, 'Induction motor', 1, 'Current sensor mux', 1);
W(p, 'Current fault delay', 1, 'Current sensor mux', 2);
W(p, 'Current sensor mux', 1, 'Current sensor a', 1);
% outputs
o = {'ia', 'ib', 'ic', 'w', 'Te', 'psir_mag'};
for k = 1:6, W(p, 'Induction motor', k, o{k}, 1); end
W(p, 'Input power sum', 1, 'P_in', 1);
W(p, 'Inverter losses', 1, 'Pel', 1);
W(p, 'Inverter losses', 2, 'Pcond', 1);
W(p, 'Inverter losses', 3, 'Psw', 1);
W(p, 'Core loss', 1, 'Pfe', 1);
W(p, 'Load power', 1, 'Pout', 1);
W(p, 'Friction power', 1, 'Pfric', 1);
W(p, 'Inverter losses', 4, 'transitions', 1);
W(p, 'Induction motor', 9, 'psim_mag', 1);
W(p, 'Mode', 1, 'mode', 1);
W(p, 'Current sensor a', 1, 'ia_m', 1);
W(p, 'Induction motor', 2, 'ib_m', 1);
W(p, 'Induction motor', 3, 'ic_m', 1);
W(p, 'Speed sensor', 1, 'w_m', 1);
FIN(p, 1400);
end

function modulator(p, pos, hvar, NTvar)
% centre-aligned PWM, double update: in a rising half-period (odd control step) a leg
% is off then on, in a falling half-period on then off (plant_step.m). The block
% outputs the pole-voltage AVERAGE over each solver step (exact volt-seconds) and
% the number of leg transitions in the step (for the switching loss).
S(p, pos, {'da', 'db', 'dc'}, {'vpa', 'vpb', 'vpc', 'trans_a', 'trans_b', 'trans_c'}, 'orange');
B([p '/Clock'], 'simulink/Sources/Clock', [20 420 50 450]);
B([p '/Step start time'], 'simulink/Discrete/Zero-Order Hold', [80 420 120 450], 'SampleTime', hvar);
F([p '/Step in carrier period'], [150 420 450 450], sprintf('floor(u/%s+0.5)-2*%s*floor(floor(u/%s+0.5)/(2*%s))', hvar, NTvar, hvar, NTvar));
W(p, 'Clock', 1, 'Step start time', 1);
W(p, 'Step start time', 1, 'Step in carrier period', 1);
legs = {'a', 'b', 'c'};
for k = 1:3
    L = legs{k};  y = 40 + 130*(k-1);
    MUX([p '/Leg ' L ' mux'], [480 y 485 y+40], 2);
    W(p, 'Step in carrier period', 1, ['Leg ' L ' mux'], 1);
    W(p, ['d' L], 1, ['Leg ' L ' mux'], 2);
    % u(1) = step m in the carrier period (0..2NT-1), rising half if m < NT; x = duty in steps
    r = sprintf('(u(1)<%s)', NTvar);  j = sprintf('(u(1)-%s*(u(1)>=%s))', NTvar, NTvar);  x = sprintf('(u(2)*%s)', NTvar);
    frac = [r '*' clampx([j '+1-' NTvar '+' x], '0', '1') '+(1-' r ')*' clampx([x '-' j], '0', '1')];
    F([p '/Leg ' L ' on-fraction'], [520 y 800 y+25], frac);
    F([p '/Leg ' L ' on at step start'], [520 y+35 800 y+60], [r '*(' j '>=' NTvar '-' x ')+(1-' r ')*(' j '<' x ')']);
    F([p '/Leg ' L ' on at step end'], [520 y+70 800 y+95], [r '*(' j '+1>' NTvar '-' x ')+(1-' r ')*(' j '+1<=' x ')']);
    B([p '/Leg ' L ' state last step'], 'simulink/Discrete/Memory', [840 y+70 880 y+100], 'InitialCondition', '0');
    MUX([p '/Leg ' L ' edge mux'], [900 y+30 905 y+100], 3);
    F([p '/Leg ' L ' transitions'], [930 y+50 1100 y+75], 'abs(u(1)-u(2))+abs(u(1)-u(3))');
    B([p '/Leg ' L ' minus half'], 'simulink/Math Operations/Bias', [830 y 860 y+25], 'Bias', '-0.5');
    G([p '/Leg ' L ' pole voltage'], [880 y 940 y+25], 'Vdc');
    for nm = {'on-fraction', 'on at step start', 'on at step end'}
        W(p, ['Leg ' L ' mux'], 1, ['Leg ' L ' ' nm{1}], 1);
    end
    W(p, ['Leg ' L ' on at step end'], 1, ['Leg ' L ' state last step'], 1);
    W(p, ['Leg ' L ' on at step start'], 1, ['Leg ' L ' edge mux'], 1);
    W(p, ['Leg ' L ' on at step end'], 1, ['Leg ' L ' edge mux'], 2);
    W(p, ['Leg ' L ' state last step'], 1, ['Leg ' L ' edge mux'], 3);
    W(p, ['Leg ' L ' edge mux'], 1, ['Leg ' L ' transitions'], 1);
    W(p, ['Leg ' L ' on-fraction'], 1, ['Leg ' L ' minus half'], 1);
    W(p, ['Leg ' L ' minus half'], 1, ['Leg ' L ' pole voltage'], 1);
    W(p, ['Leg ' L ' pole voltage'], 1, ['vp' L], 1);
    W(p, ['Leg ' L ' transitions'], 1, ['trans_' L], 1);
end
FIN(p, 1200);
end

function inverter_losses(p, pos, hvar)
% electrical power into the motor (pole voltages x phase currents), IGBT/diode
% conduction loss, switching loss (Esw scaled with Vdc and |i| per transition)
nm = {'vpa', 'vpb', 'vpc', 'ia', 'ib', 'ic', 'trans_a', 'trans_b', 'trans_c'};
S(p, pos, nm, {'Pel', 'Pcond', 'Psw', 'transitions'}, 'orange');
MUX([p '/Inverter mux'], [120 30 125 600], 9);
for k = 1:9, W(p, nm{k}, 1, 'Inverter mux', k); end
F([p '/Pel'], [180 60 420 90], 'u(1)*u(4)+u(2)*u(5)+u(3)*u(6)');
F([p '/Pcond'], [180 160 520 190], 'Vce0*(abs(u(4))+abs(u(5))+abs(u(6)))+rce*(u(4)*u(4)+u(5)*u(5)+u(6)*u(6))');
% switching loss: the current at the start of the solver step in which the leg switches
% (for DTC exactly the switching instant, as in plant_step.m)
B([p '/Currents at step start'], 'simulink/Discrete/Zero-Order Hold', [140 640 180 670], 'SampleTime', hvar);
B([p '/Currents mux'], 'simulink/Signal Routing/Mux', [100 640 105 700], 'Inputs', '3');
for k = 1:3, W(p, nm{3 + k}, 1, 'Currents mux', k); end
W(p, 'Currents mux', 1, 'Currents at step start', 1);
MUX([p '/Switching mux'], [200 620 205 720], 2);
W(p, 'Currents at step start', 1, 'Switching mux', 1);
B([p '/Transitions mux'], 'simulink/Signal Routing/Mux', [100 720 105 780], 'Inputs', '3');
for k = 1:3, W(p, nm{6 + k}, 1, 'Transitions mux', k); end
W(p, 'Transitions mux', 1, 'Switching mux', 2);
F([p '/Psw'], [240 655 680 685], sprintf('Esw*(Vdc/Vref_sw)/Iref_sw*(u(4)*abs(u(1))+u(5)*abs(u(2))+u(6)*abs(u(3)))/%s', hvar));
W(p, 'Switching mux', 1, 'Psw', 1);
F([p '/Transition count'], [180 360 420 390], 'u(7)+u(8)+u(9)');
o = {'Pel', 'Pcond', 'Transition count'};
for k = 1:3, W(p, 'Inverter mux', 1, o{k}, 1); end
W(p, 'Pel', 1, 'Pel out', 1);
W(p, 'Pcond', 1, 'Pcond out', 1);
W(p, 'Psw', 1, 'Psw out', 1);
W(p, 'Transition count', 1, 'transitions out', 1);
FIN(p, 700);
end

function core_loss(p, pos)
% P_fe = (kh*|fe| + ke*fe^2)*|psi_m|^2, fe from the rotation of psi_m over one Ts (plant_step.m)
S(p, pos, {'psim_alpha', 'psim_beta'}, {'Pfe'}, 'orange');
B([p '/psim_alpha sample'], 'simulink/Discrete/Zero-Order Hold', [100 30 140 60], 'SampleTime', 'Ts');
B([p '/psim_beta sample'], 'simulink/Discrete/Zero-Order Hold', [100 100 140 130], 'SampleTime', 'Ts');
W(p, 'psim_alpha', 1, 'psim_alpha sample', 1);
W(p, 'psim_beta', 1, 'psim_beta sample', 1);
STATES(p, {'flux angle', '0'});
FXS(p, 'flux angle', 'atan2(b,a)', {'a', 'psim_alpha sample', 'b', 'psim_beta sample'});
FX(p, 'angle step', 'g-gp', {'g', 'flux angle', 'gp', 'flux angle(k-1)'});
FX(p, 'fe', '(d-twopi*floor(d/twopi+0.5))/(twopi*Ts)', {'d', 'angle step'});
FX(p, 'Pfe', '(kh*abs(f)+ke*f*f)*(a*a+b*b)', {'f', 'fe', 'a', 'psim_alpha sample', 'b', 'psim_beta sample'});
W(p, 'Pfe', 1, 'Pfe out', 1);
FIN(p);
end

function induction_motor(p, pos)
% saturated induction motor, stationary alpha-beta frame, flux linkages as states
% (plant_step.m / im_currents.m):
%   g_a = 1/Lls (mode 1), 0 (mode 2), 1/(3 Lls) (mode 3);  g_b = 1/Lls;  g_r = 1/Llr
%   psi_m = (g_s*psi_s + g_r*psi_r)/(1/Lms(|psi_m|) + g_s + g_r)   (fixed point, warm start)
%   i_s = g_s*(psi_s - psi_m),  i_r = g_r*(psi_r - psi_m),  Te = 1.5 p (psi_m x i_s)
S(p, pos, {'vpa', 'vpb', 'vpc', 'TL', 'J_extra', 'mode'}, ...
    {'ia', 'ib', 'ic', 'w', 'Te', 'psir_mag', 'psim_alpha', 'psim_beta', 'psim_mag'}, 'lightBlue');
st = {'lam_alpha', 'psi_s_beta', 'psi_r_alpha', 'psi_r_beta', 'omega_m'};
for k = 1:5
    [xx, yy] = LPOS(p);
    if k == 1
        % entering mode 3 sets lam_alpha = psi_m_alpha (keeps i_alpha = 0 at the switch)
        B([p '/' st{k}], 'simulink/Continuous/Integrator', [xx yy xx+50 yy+50], ...
            'ExternalReset', 'rising', 'InitialConditionSource', 'external');
    else
        B([p '/' st{k}], 'simulink/Continuous/Integrator', [xx yy xx+40 yy+30]);
    end
end
FX(p, 'g_a', '(m<1.5)/Lls+(m>2.5)/(3*Lls)', {'m', 'mode'});
% saturation: 4 fixed-point iterations from the last solver step's psi_m
B([p '/psim_alpha last step'], 'simulink/Discrete/Memory', [20 900 60 930], 'InitialCondition', '0');
B([p '/psim_beta last step'], 'simulink/Discrete/Memory', [20 960 60 990], 'InitialCondition', '0');
prev_a = 'psim_alpha last step';  prev_b = 'psim_beta last step';
Lsat = 'Lm_rated+sat_on*(Lm0/(1+sat_a*(sqrt(pa*pa+pb*pb)/psi_k)^sat_b)-Lm_rated)';
for it = 1:4
    Ln = sprintf('Lms iteration %d', it);
    FX(p, Ln, Lsat, {'pa', prev_a, 'pb', prev_b});
    FX(p, sprintf('psim_alpha iteration %d', it), '(ga*x1+x3/Llr)/(1/L+ga+1/Llr)', {'ga', 'g_a', 'x1', st{1}, 'x3', st{3}, 'L', Ln});
    FX(p, sprintf('psim_beta iteration %d', it), '(x2/Lls+x4/Llr)/(1/L+1/Lls+1/Llr)', {'x2', st{2}, 'x4', st{4}, 'L', Ln});
    prev_a = sprintf('psim_alpha iteration %d', it);  prev_b = sprintf('psim_beta iteration %d', it);
end
W(p, prev_a, 1, 'psim_alpha last step', 1);
W(p, prev_b, 1, 'psim_beta last step', 1);
% reset value uses the inductance of the first iteration (Memory-based): no algebraic loop
FX(p, 'psim_alpha phase open', '(x3/Llr)/(1/L+1/Llr)', {'x3', st{3}, 'L', 'Lms iteration 1'});
FX(p, 'i_s_alpha', 'ga*(x1-pa)', {'ga', 'g_a', 'x1', st{1}, 'pa', prev_a});
FX(p, 'i_s_beta', '(x2-pb)/Lls', {'x2', st{2}, 'pb', prev_b});
FX(p, 'i_r_alpha', '(x3-pa)/Llr', {'x3', st{3}, 'pa', prev_a});
FX(p, 'i_r_beta', '(x4-pb)/Llr', {'x4', st{4}, 'pb', prev_b});
FX(p, 'Te', '1.5*pp*(pa*ib-pb*ia)', {'pa', prev_a, 'pb', prev_b, 'ia', 'i_s_alpha', 'ib', 'i_s_beta'});
% stator voltages: mode 1 Clarke; mode 3 -(vb+vc) on the alpha axis; mode 2 alpha unused
FX(p, 'v_alpha', '(m<1.5)*(2/3)*(a-0.5*(b+c))+(m>2.5)*(-(b+c))', {'m', 'mode', 'a', 'vpa', 'b', 'vpb', 'c', 'vpc'});
FX(p, 'v_beta', '(b-c)/sqrt(3)', {'b', 'vpb', 'c', 'vpc'});
FX(p, 'd lam_alpha', '(m<1.5)*(v-Rs*i)+(m>2.5)*(v-3*Rs*i)', {'m', 'mode', 'v', 'v_alpha', 'i', 'i_s_alpha'});
FX(p, 'd psi_s_beta', 'v-Rs*i', {'v', 'v_beta', 'i', 'i_s_beta'});
FX(p, 'd psi_r_alpha', '-Rr*i-pp*w*x4', {'i', 'i_r_alpha', 'w', st{5}, 'x4', st{4}});
FX(p, 'd psi_r_beta', '-Rr*i+pp*w*x3', {'i', 'i_r_beta', 'w', st{5}, 'x3', st{3}});
FX(p, 'd omega_m', '(T-L-Bm*w)/(Jm+J)', {'T', 'Te', 'L', 'TL', 'w', st{5}, 'J', 'J_extra'});
FX(p, 'mode 3 entered', 'm>2.5', {'m', 'mode'});
W(p, 'd lam_alpha', 1, st{1}, 1);
W(p, 'mode 3 entered', 1, st{1}, 2);
W(p, 'psim_alpha phase open', 1, st{1}, 3);
W(p, 'd psi_s_beta', 1, st{2}, 1);
W(p, 'd psi_r_alpha', 1, st{3}, 1);
W(p, 'd psi_r_beta', 1, st{4}, 1);
W(p, 'd omega_m', 1, st{5}, 1);
% phase currents (amplitude-invariant Clarke; mode 3: i0 = -i_alpha so ia = 0)
FX(p, 'ia', '(m<1.5)*ia', {'m', 'mode', 'ia', 'i_s_alpha'});
FX(p, 'ib', '-(0.5+(m>2.5))*ia+s32*ib', {'m', 'mode', 'ia', 'i_s_alpha', 'ib', 'i_s_beta'});
FX(p, 'ic', '-(0.5+(m>2.5))*ia-s32*ib', {'m', 'mode', 'ia', 'i_s_alpha', 'ib', 'i_s_beta'});
FX(p, 'psi_r magnitude', 'sqrt(a*a+b*b)', {'a', st{3}, 'b', st{4}});
FX(p, 'psi_m magnitude', 'sqrt(a*a+b*b)', {'a', prev_a, 'b', prev_b});
o = {'ia', 'ib', 'ic', st{5}, 'Te', 'psi_r magnitude', prev_a, prev_b, 'psi_m magnitude'};
on = {'ia', 'ib', 'ic', 'w', 'Te', 'psir_mag', 'psim_alpha', 'psim_beta', 'psim_mag'};
for k = 1:9, W(p, o{k}, 1, [on{k} ' out'], 1); end
FIN(p);
end

%% =====================================================================
%  AGV mission model: two wheel drives + agv_step.m as blocks
%  =====================================================================
function build_agv_model(here, type)
mdl = ['AGV_' type '_BLOCKS'];
evalin('base', sprintf('bm_drive = ''%s''; bm_model = ''''; block_model_params;', type));
file = new_model(mdl, here, 'h_step_agv', 't_stop_agv', type, sprintf([ ...
    'AGV warehouse mission with two %s wheel drives (agv_step.m drawn as blocks, thesis parameters P).\n' ...
    'Edit on this level: Flux select (1 rated, 2 fixed, 3 LUT, 4 MLP), LUT source (1 analytic, 2 ML),\n' ...
    'Fault type (0 none, 1 speed sensor, 2 current sensor, 3 open phase), Fault side (1 left, 2 right).\n' ...
    'Route, mass, speeds, fault time: P.agv (block_model_params.m). The run stops 0.5 s after the mission ends.'], type), [30 1000 900 1080]);
B([mdl '/Flux select'], 'simulink/Sources/Constant', [20 100 70 130], 'Value', 'flux_mode_default');
B([mdl '/LUT source'], 'simulink/Sources/Constant', [20 170 70 200], 'Value', '1');
B([mdl '/Speed controller'], 'simulink/Sources/Constant', [20 240 70 270], 'Value', '1');
B([mdl '/Fault type'], 'simulink/Sources/Constant', [20 560 70 590], 'Value', 'agv_fault_type_default');
B([mdl '/Fault side'], 'simulink/Sources/Constant', [20 620 70 650], 'Value', 'agv_fault_side_default');
agv_mission([mdl '/AGV mission and supervisor'], [200 60 400 700]);
sides = {'Left', 'Right'};
for k = 1:2
    sd = [sides{k} ' wheel drive'];
    wheel_drive([mdl '/' sd], [600 60 + 420*(k-1) 800 400 + 420*(k-1)], type);
    % wheel drive in: 1 w_ref 2 TL 3 J_extra 4 f_speed 5 f_cur 6 f_open 7 flux_sel 8 lut_sel 9 speed_mode
    for j = 1:6, W(mdl, 'AGV mission and supervisor', 6*(k-1) + j, sd, j); end
    W(mdl, 'Flux select', 1, sd, 7);
    W(mdl, 'LUT source', 1, sd, 8);
    W(mdl, 'Speed controller', 1, sd, 9);
end
% mission in: 1 fault type 2 fault side 3..8 left (w E_in f_speed f_cur f_open neutral) 9..14 right
W(mdl, 'Fault type', 1, 'AGV mission and supervisor', 1);
W(mdl, 'Fault side', 1, 'AGV mission and supervisor', 2);
for k = 1:2
    for j = 1:6
        W(mdl, [sides{k} ' wheel drive'], j, 'AGV mission and supervisor', 2 + 6*(k-1) + j);
    end
end
% mission out 13..25: x y v v_cmd E_Wh e_cross phase limp stop done s e_max t_mission
A = 'AGV mission and supervisor';
x0 = 1000;
B([mdl '/Route (x-y)'], 'simulink/Sinks/XY Graph', [x0 60 x0+40 100]);
W(mdl, A, 13, 'Route (x-y)', 1);
W(mdl, A, 14, 'Route (x-y)', 2);
SCOPE(mdl, 'Robot speed (m_s)', [x0 150], {{A, 16}, {A, 15}});
SCOPE(mdl, 'Mission state', [x0 250], {{A, 19}, {A, 20}, {A, 21}, {A, 22}});
SCOPE(mdl, 'Cross-track error (m)', [x0 350], {{A, 18}});
SCOPE(mdl, 'Fault flags', [x0 450], {{'Left wheel drive', 3}, {'Left wheel drive', 4}, {'Left wheel drive', 5}, ...
    {'Right wheel drive', 3}, {'Right wheel drive', 4}, {'Right wheel drive', 5}});
SCOPE(mdl, 'Motor speeds (rad_s)', [x0 550], {{'Left wheel drive', 1}, {'Right wheel drive', 1}});
B([mdl '/Battery energy (Wh)'], 'simulink/Sinks/Display', [x0 660 x0+100 690]);
W(mdl, A, 17, 'Battery energy (Wh)', 1);
B([mdl '/Mission time (s)'], 'simulink/Sinks/Display', [x0 720 x0+100 750]);
W(mdl, A, 25, 'Mission time (s)', 1);
G([mdl '/m to cm'], [x0 780 x0+40 810], '100');
B([mdl '/Max path error (cm)'], 'simulink/Sinks/Display', [x0+60 780 x0+160 810]);
W(mdl, A, 24, 'm to cm', 1);
W(mdl, 'm to cm', 1, 'Max path error (cm)', 1);
MUX([mdl '/Log mux'], [x0+300 300 x0+305 600], 13);
for k = 1:13, W(mdl, A, 12 + k, 'Log mux', k); end
B([mdl '/Log to workspace'], 'simulink/Sinks/To Workspace', [x0+350 430 x0+450 460], ...
    'VariableName', 'agv_log', 'SaveFormat', 'Structure With Time', 'Decimation', '40', 'MaxDataPoints', 'inf');
W(mdl, 'Log mux', 1, 'Log to workspace', 1);
finish_model(mdl, file);
end

function wheel_drive(p, pos, type)
% one complete wheel drive: controller (ctrl_step) + inverter and motor (plant_step)
S(p, pos, {'w_ref', 'TL', 'J_extra', 'f_speed_inj', 'f_cur_inj', 'f_open_inj', 'flux_sel', 'lut_sel', 'speed_mode'}, ...
    {'w', 'E_in', 'f_speed', 'f_cur', 'f_open', 'neutral'}, 'lightBlue');
drive_core(p, type, 'h_step_agv', 'NT_agv', [300 60], {'w_ref', 1}, {'TL', 1}, {'J_extra', 1}, ...
    {'f_speed_inj', 1}, {'f_cur_inj', 1}, {'f_open_inj', 1}, 'flux_sel', 'lut_sel', 'speed_mode');
B([p '/Input energy'], 'simulink/Continuous/Integrator', [1160 120 1200 150]);
W(p, 'Inverter and motor', 7, 'Input energy', 1);
W(p, 'Inverter and motor', 4, 'w', 1);
W(p, 'Input energy', 1, 'E_in', 1);
W(p, 'Controller', 11, 'f_speed', 1);
W(p, 'Controller', 12, 'f_cur', 1);
W(p, 'Controller', 13, 'f_open', 1);
W(p, 'Controller', 4, 'neutral', 1);
% signals the vehicle does not need (open a scope here to look at them)
unused_c = [5 6 7 8 9 10];  unused_p = [1 2 3 5 6 8 9 10 11 12 13 14 15 16];
for k = unused_c
    nm = sprintf('Controller out %d not used', k);
    B([p '/' nm], 'simulink/Sinks/Terminator', [560 500 + 25*k 580 515 + 25*k]);
    W(p, 'Controller', k, nm, 1);
end
for k = unused_p
    nm = sprintf('Plant out %d not used', k);
    B([p '/' nm], 'simulink/Sinks/Terminator', [1160 500 + 25*k 1180 515 + 25*k]);
    W(p, 'Inverter and motor', k, nm, 1);
end
FIN(p, 1400);
end

function agv_mission(p, pos)
% agv_step.m every Ts: kinematics from the measured motor speeds, battery energy,
% progress and cross-track error, fault supervisor, mission logic, pure pursuit,
% wheel speed references, wheel loads, fault injection.
S(p, pos, {'fault_type', 'fault_side', 'wL', 'EL', 'fsL', 'fcL', 'foL', 'nL', 'wR', 'ER', 'fsR', 'fcR', 'foR', 'nR'}, ...
    {'wL_ref', 'TL_L', 'J_L', 'fs_L', 'fc_L', 'fo_L', 'wR_ref', 'TL_R', 'J_R', 'fs_R', 'fc_R', 'fo_R', ...
     'x', 'y', 'v', 'v_cmd', 'E_Wh', 'e_cross', 'phase', 'limp', 'stop', 'done', 's', 'e_max', 't_mission'}, 'magenta');
% motor speeds and energies sampled now; drive flags of the last sample (as the thesis model)
smp = {'wL', 'EL', 'wR', 'ER'};
for k = 1:4
    yy = 30 + 50*(k-1);
    B([p '/' smp{k} ' sample'], 'simulink/Discrete/Zero-Order Hold', [120 yy 160 yy+30], 'SampleTime', 'Ts');
    W(p, smp{k}, 1, [smp{k} ' sample'], 1);
end
dl = {'fsL', 'fcL', 'foL', 'nL', 'fsR', 'fcR', 'foR', 'nR'};
for k = 1:8
    yy = 250 + 50*(k-1);
    B([p '/' dl{k} '(k-1)'], 'simulink/Discrete/Unit Delay', [120 yy 160 yy+30], 'SampleTime', 'Ts', 'InitialCondition', '0');
    W(p, dl{k}, 1, [dl{k} '(k-1)'], 1);
end
STATES(p, {'t', '0'; 'px', '0'; 'py', '0'; 'heading', '0'; 's', '0'; 'e_max', '0'; 'limp', '0'; 'stop', '0'; ...
    'dwell', '0'; 'phase', '1'; 'done', '0'; 'm_tot', 'm_robot+m_payload'; 'v_cmd', '0'; 'mission time', '0'; 'after done', '0'});
FXS(p, 't', 'tp+Ts', {'tp', 't(k-1)'});
% kinematics (Euler, as agv_step.m)
FX(p, 'v', 'r_wheel*(wR/G_gear+wL/G_gear)/2', {'wR', 'wR sample', 'wL', 'wL sample'});
FX(p, 'om', 'r_wheel*(wR/G_gear-wL/G_gear)/W_track', {'wR', 'wR sample', 'wL', 'wL sample'});
FXS(p, 'px', 'x+Ts*v*cos(h)', {'x', 'px(k-1)', 'v', 'v', 'h', 'heading(k-1)'});
FXS(p, 'py', 'y+Ts*v*sin(h)', {'y', 'py(k-1)', 'v', 'v', 'h', 'heading(k-1)'});
FXS(p, 'heading', 'h+Ts*om', {'h', 'heading(k-1)', 'om', 'om'});
FX(p, 'E_Wh', '(a+b)/3600', {'a', 'EL sample', 'b', 'ER sample'});
% closest point on the rectangle, searched in [s_prev-0.5, s_prev+1]; first strict minimum wins
seg = {'0', '0', 'Lx', '0'; 'Lx', '0', 'Lx', 'Ly'; 'Lx', 'Ly', '0', 'Ly'; '0', 'Ly', '0', '0'};
s0 = {'0', 'Lx', 'Lx+Ly', '2*Lx+Ly'};
for k = 1:4
    x0 = seg{k,1}; y0 = seg{k,2}; x1 = seg{k,3}; y1 = seg{k,4};
    dx = sprintf('(%s-%s)', x1, x0);  dy = sprintf('(%s-%s)', y1, y0);
    L2 = sprintf('(%s*%s+%s*%s)', dx, dx, dy, dy);
    f = clampx(sprintf('((x-%s)*%s+(y-%s)*%s)/%s', x0, dx, y0, dy, L2), '0', '1');
    FX(p, sprintf('segment %d fraction', k), f, {'x', 'px', 'y', 'py'});
    FX(p, sprintf('segment %d progress', k), sprintf('%s+f*sqrt(%s)', s0{k}, L2), {'f', sprintf('segment %d fraction', k)});
    FX(p, sprintf('segment %d distance', k), sprintf(['sqrt((x-(%s+f*%s))*(x-(%s+f*%s))+(y-(%s+f*%s))*(y-(%s+f*%s)))' ...
        '+1e9*(((s<sp-0.5)+(s>sp+1))>0)'], x0, dx, x0, dx, y0, dy, y0, dy), ...
        {'x', 'px', 'y', 'py', 'f', sprintf('segment %d fraction', k), 's', sprintf('segment %d progress', k), 'sp', 's(k-1)'});
    FX(p, sprintf('segment %d signed error', k), sprintf('(%s*(y-(%s+f*%s))-%s*(x-(%s+f*%s)))/sqrt(%s)', dx, y0, dy, dy, x0, dx, L2), ...
        {'x', 'px', 'y', 'py', 'f', sprintf('segment %d fraction', k)});
end
dd = {'d1', 'segment 1 distance', 'd2', 'segment 2 distance', 'd3', 'segment 3 distance', 'd4', 'segment 4 distance'};
FX(p, 'segment winner 1', '(d1<1e8)*(d1<=d2)*(d1<=d3)*(d1<=d4)', dd);
FX(p, 'segment winner 2', '(d2<1e8)*(d2<d1)*(d2<=d3)*(d2<=d4)', dd);
FX(p, 'segment winner 3', '(d3<1e8)*(d3<d1)*(d3<d2)*(d3<=d4)', dd);
FX(p, 'segment winner 4', '(d4<1e8)*(d4<d1)*(d4<d2)*(d4<d3)', dd);
ww = {'b1', 'segment winner 1', 'b2', 'segment winner 2', 'b3', 'segment winner 3', 'b4', 'segment winner 4'};
FXS(p, 's', mx(ite('(b1+b2+b3+b4)>0.5', 'b1*s1+b2*s2+b3*s3+b4*s4', 'sp'), 'sp'), [ww, {'s1', 'segment 1 progress', ...
    's2', 'segment 2 progress', 's3', 'segment 3 progress', 's4', 'segment 4 progress', 'sp', 's(k-1)'}]);
FX(p, 'e_cross', 'b1*e1+b2*e2+b3*e3+b4*e4', [ww, {'e1', 'segment 1 signed error', 'e2', 'segment 2 signed error', ...
    'e3', 'segment 3 signed error', 'e4', 'segment 4 signed error'}]);
FXS(p, 'e_max', mx('em', 'abs(e)'), {'em', 'e_max(k-1)', 'e', 'e_cross'});
% fault supervisor (flags of the last sample of both drives)
FX(p, 'any fault', '(fa+fb+fc+fd+fe+ff)>0.5', {'fa', 'fsL(k-1)', 'fb', 'fcL(k-1)', 'fc', 'foL(k-1)', 'fd', 'fsR(k-1)', 'fe', 'fcR(k-1)', 'ff', 'foR(k-1)'});
FX(p, 'open phase without remedy', '((oL>0.5)*(nL<0.5)+(oR>0.5)*(nR<0.5))>0', {'oL', 'foL(k-1)', 'nL', 'nL(k-1)', 'oR', 'foR(k-1)', 'nR', 'nR(k-1)'});
FXS(p, 'limp', mx('l', 'a'), {'l', 'limp(k-1)', 'a', 'any fault'});
FXS(p, 'stop', mx('st', 'o'), {'st', 'stop(k-1)', 'o', 'open phase without remedy'});
% mission logic: phase 1 to B, 2 dwell at B, 3 back to A, 4 done
FX(p, 'd_goal', mx([ite('ph<1.5', 'sB', 'sEnd') '-s'], '0'), {'ph', 'phase(k-1)', 's', 's'});
FX(p, 'arrive B', '(ph>0.5)*(ph<1.5)*(d<0.05)*(abs(v)<0.02)', {'ph', 'phase(k-1)', 'd', 'd_goal', 'v', 'v'});
FXS(p, 'dwell', ite('a', '0', ite('(ph>1.5)*(ph<2.5)', 'dw+Ts', 'dw')), {'a', 'arrive B', 'ph', 'phase(k-1)', 'dw', 'dwell(k-1)'});
FX(p, 'leave B', '(ph>1.5)*(ph<2.5)*(dw>=dwell_t)', {'ph', 'phase(k-1)', 'dw', 'dwell'});
FX(p, 'arrive A', '(ph>2.5)*(ph<3.5)*(d<0.05)*(abs(v)<0.02)*(dn<1.5)', {'ph', 'phase(k-1)', 'd', 'd_goal', 'v', 'v', 'dn', 'done(k-1)'});
FXS(p, 'phase', 'ph+a+l+r', {'ph', 'phase(k-1)', 'a', 'arrive B', 'l', 'leave B', 'r', 'arrive A'});
FXS(p, 'done', mx('dn', 'r'), {'dn', 'done(k-1)', 'r', 'arrive A'});
FXS(p, 'm_tot', ite('l', 'm_robot', 'm'), {'l', 'leave B', 'm', 'm_tot(k-1)'});
% pure pursuit
FX(p, 'look-ahead s', mn('s+L_look', 'sEnd'), {'s', 's'});
[xx, yy] = LPOS(p);
B([p '/Route x'], 'simulink/Lookup Tables/1-D Lookup Table', [xx yy xx+70 yy+40], 'Table', '[0 Lx Lx 0 0]', ...
    'BreakpointsForDimension1', '[0 Lx Lx+Ly 2*Lx+Ly 2*(Lx+Ly)]', 'ExtrapMethod', 'Clip');
[xx, yy] = LPOS(p);
B([p '/Route y'], 'simulink/Lookup Tables/1-D Lookup Table', [xx yy xx+70 yy+40], 'Table', '[0 0 Ly Ly 0]', ...
    'BreakpointsForDimension1', '[0 Lx Lx+Ly 2*Lx+Ly 2*(Lx+Ly)]', 'ExtrapMethod', 'Clip');
W(p, 'look-ahead s', 1, 'Route x', 1);
W(p, 'look-ahead s', 1, 'Route y', 1);
FX(p, 'alpha', '(atan2(gy-y,gx-x)-h)-twopi*floor((atan2(gy-y,gx-x)-h)/twopi+0.5)', {'gx', 'Route x', 'gy', 'Route y', 'x', 'px', 'y', 'py', 'h', 'heading'});
FX(p, 'kappa', '2*sin(a)/L_look', {'a', 'alpha'});
FX(p, 'v_corner', ['sqrt(a_lat/' mx('abs(k)', '1e-3') ')'], {'k', 'kappa'});
FX(p, 'v_target', ite('((ph>1.5)*(ph<2.5)+(ph>3.5)+(st>0)+(t<t_start))>0', '0', ...
    mn(mn(ite('l>0', mn('v_max', 'v_limp'), 'v_max'), 'sqrt(2*0.8*a_max*d)'), 'vc')), ...
    {'ph', 'phase', 'st', 'stop', 't', 't', 'l', 'limp', 'd', 'd_goal', 'vc', 'v_corner'});
FXS(p, 'v_cmd', ['vp+' clampx('vt-vp', '-a_max*Ts', 'a_max*Ts')], {'vp', 'v_cmd(k-1)', 'vt', 'v_target'});
FX(p, 'wL_ref', 'G_gear*(v-k*v*W_track/2)/r_wheel', {'v', 'v_cmd', 'k', 'kappa'});
FX(p, 'wR_ref', 'G_gear*(v+k*v*W_track/2)/r_wheel', {'v', 'v_cmd', 'k', 'kappa'});
% loads and reflected inertia
FX(p, 'TL_L', 'r_wheel/(G_gear*eta_g)*Crr*m*9.81/2*tanh(w/G_gear/0.2)', {'m', 'm_tot', 'w', 'wL sample'});
FX(p, 'TL_R', 'r_wheel/(G_gear*eta_g)*Crr*m*9.81/2*tanh(w/G_gear/0.2)', {'m', 'm_tot', 'w', 'wR sample'});
FX(p, 'J_extra', '(r_wheel/G_gear)*(r_wheel/G_gear)*m/2', {'m', 'm_tot'});
% fault injection (experiment definition)
for k = 1:3
    FX(p, sprintf('left fault %d', k), sprintf('(abs(ft-%d)<0.5)*(abs(fs-1)<0.5)*(t>=agv_fault_time)', k), {'ft', 'fault_type', 'fs', 'fault_side', 't', 't'});
    FX(p, sprintf('right fault %d', k), sprintf('(abs(ft-%d)<0.5)*(abs(fs-2)<0.5)*(t>=agv_fault_time)', k), {'ft', 'fault_type', 'fs', 'fault_side', 't', 't'});
end
% mission time; stop 0.5 s after the mission ends
FXS(p, 'mission time', ite('dp>0.5', 'tm', 't'), {'dp', 'done(k-1)', 'tm', 'mission time(k-1)', 't', 't'});
FXS(p, 'after done', 'a+Ts*(d>0.5)', {'a', 'after done(k-1)', 'd', 'done'});
FX(p, 'end of run', 'a>=0.5', {'a', 'after done'});
[xx, yy] = LPOS(p);
B([p '/Stop'], 'simulink/Sinks/Stop Simulation', [xx yy xx+30 yy+30]);
W(p, 'end of run', 1, 'Stop', 1);
% outputs
o = {'wL_ref', 'TL_L', 'J_extra', 'left fault 1', 'left fault 2', 'left fault 3', ...
     'wR_ref', 'TL_R', 'J_extra', 'right fault 1', 'right fault 2', 'right fault 3', ...
     'px', 'py', 'v', 'v_cmd', 'E_Wh', 'e_cross', 'phase', 'limp', 'stop', 'done', 's', 'e_max', 'mission time'};
on = {'wL_ref', 'TL_L', 'J_L', 'fs_L', 'fc_L', 'fo_L', 'wR_ref', 'TL_R', 'J_R', 'fs_R', 'fc_R', 'fo_R', ...
      'x', 'y', 'v', 'v_cmd', 'E_Wh', 'e_cross', 'phase', 'limp', 'stop', 'done', 's', 'e_max', 't_mission'};
for k = 1:numel(o), W(p, o{k}, 1, [on{k} ' out'], 1); end
FIN(p);
end

%% =====================================================================
%  Expression helpers (Fcn blocks have no min/max/if: use 0/1 arithmetic)
%  =====================================================================
function s = mx(a, b), s = sprintf('(((%s)>(%s))*(%s)+((%s)<=(%s))*(%s))', a, b, a, a, b, b); end
function s = mn(a, b), s = sprintf('(((%s)<(%s))*(%s)+((%s)>=(%s))*(%s))', a, b, a, a, b, b); end
function s = clampx(x, lo, hi), s = mn(mx(x, lo), hi); end
function s = ite(c, a, b), s = sprintf('((%s)*(%s)+(1-(%s))*(%s))', c, a, c, b); end

%% =====================================================================
%  Block helpers
%  =====================================================================
function FX(sys, name, expr, ins)
% Fcn block "name" computing expr; ins = {alias1, ref1, alias2, ref2, ...};
% the aliases in expr become u(1), u(2), ... A ref is a block name (port 1) or {block, port}.
n = numel(ins)/2;
for i = 1:n
    expr = regexprep(expr, ['(?<![A-Za-z0-9_])' ins{2*i-1} '(?![A-Za-z0-9_])'], sprintf('u(%d)', i));
end
if n == 1, expr = strrep(expr, 'u(1)', 'u'); end
[xx, yy] = LPOS(sys);
F([sys '/' name], [xx+50 yy xx+430 yy+30], expr);
if n == 1
    W2(sys, ref2(ins{2}), name, 1);
else
    MUX([sys '/' name ' in'], [xx yy-10 xx+5 yy+40], n);
    for i = 1:n, W2(sys, ref2(ins{2*i}), [name ' in'], i); end
    W(sys, [name ' in'], 1, name, 1);
end
end

function FXS(sys, name, expr, ins)
% controller state update: Fcn "name" = new value -> Unit Delay "name(k-1)" (created by STATES)
[xx, yy] = LPOS(sys, 'peek');
set_param([sys '/' name '(k-1)'], 'Position', [xx+470 yy xx+530 yy+30]);
FX(sys, name, expr, ins);
W(sys, name, 1, [name '(k-1)'], 1);
end

function STATES(sys, list)
% create the Unit Delays "x(k-1)" of all states of a subsystem first, so any
% block can read a state before its update is drawn
for i = 1:size(list, 1)
    B([sys '/' list{i,1} '(k-1)'], 'simulink/Discrete/Unit Delay', [3000 30+40*i 3060 60+40*i], ...
        'SampleTime', 'Ts', 'InitialCondition', list{i,2});
end
end

function r = ref2(r)
if ischar(r), r = {r, 1}; end
end

function [x, y] = LPOS(sys, mode)
% grid layout: rows of 70 px, 15 rows per column, columns 620 px apart
persistent M
if isempty(M), M = containers.Map(); end
if nargin > 1 && strcmp(mode, 'reset'), M = containers.Map(); x = 0; y = 0; return; end
if ~isKey(M, sys), M(sys) = 0; end
n = M(sys);
x = 220 + 620*floor(n/15);
y = 30 + 70*mod(n, 15);
if nargin < 2 || ~strcmp(mode, 'peek'), M(sys) = n + 1; end
end

function FIN(sys, xout)
% move the outports to the right of the last column
if nargin < 2
    [x, ~] = LPOS(sys, 'peek');
    xout = x + 640;
end
try
    h = find_system(sys, 'SearchDepth', 1, 'BlockType', 'Outport');
catch
    h = {};
end
for i = 1:numel(h)
    set_param(h{i}, 'Position', [xout 30+60*(i-1) xout+30 44+60*(i-1)]);
end
end

function file = new_model(mdl, here, step, stop, type, note, note_pos)
file = fullfile(here, [mdl '.slx']);
if bdIsLoaded(mdl), close_system(mdl, 0); end
if exist(file, 'file'), delete(file); end
LPOS('', 'reset');
new_system(mdl);
set_param(mdl, 'SolverType', 'Fixed-step', 'Solver', 'ode4', 'FixedStep', step, ...
    'StopTime', stop, 'ReturnWorkspaceOutputs', 'on', 'SaveTime', 'off', ...
    'SaveOutput', 'off', 'SignalLogging', 'off', 'EnableMultiTasking', 'off');
% parameters are reloaded before every run, so edits to block_model_params.m always apply
set_param(mdl, 'InitFcn', sprintf(['addpath(fileparts(get_param(''%s'',''FileName''))); ' ...
    'bm_drive = ''%s''; bm_model = ''%s''; block_model_params;'], mdl, type, mdl));
try
    a = Simulink.Annotation(mdl, note);
    a.Position = note_pos;
catch
end
end

function finish_model(mdl, file)
save_system(mdl, file);
open_system(mdl);
fprintf('Saved %s\n', file);
end

function S(path, pos, ins, outs, color)
% subsystem with named ports; the outports are named "<name> out"
add_block('built-in/Subsystem', path, 'Position', pos);
for i = 1:numel(ins)
    y = 30 + 50*(i-1);
    add_block('built-in/Inport', [path '/' ins{i}], 'Port', num2str(i), 'Position', [20 y 50 y+14]);
end
for i = 1:numel(outs)
    y = 30 + 60*(i-1);
    add_block('built-in/Outport', [path '/' outs{i} ' out'], 'Port', num2str(i), 'Position', [3000 y 3030 y+14]);
end
if ~isempty(color), set_param(path, 'BackgroundColor', color); end
end

function h = B(path, lib, pos, varargin)
h = add_block(lib, path, 'Position', pos);
for k = 1:2:numel(varargin)
    try
        set_param(h, varargin{k}, varargin{k+1});
    catch err
        warning('build_block_models:param', '%s: could not set %s = %s (%s)', ...
            path, varargin{k}, varargin{k+1}, err.message);
    end
end
end

function F(path, pos, expr)
B(path, 'simulink/User-Defined Functions/Fcn', pos, 'Expr', expr);
end

function G(path, pos, gain)
B(path, 'simulink/Math Operations/Gain', pos, 'Gain', gain);
end

function SUM(path, pos, signs)
B(path, 'simulink/Math Operations/Sum', pos, 'Inputs', signs, 'IconShape', 'rectangular');
end

function MUX(path, pos, n)
B(path, 'simulink/Signal Routing/Mux', pos, 'Inputs', num2str(n));
end

function W(sys, src, sp, dst, dp)
% connect output port sp of block src to input port dp of block dst; a subsystem's
% own outports are "<name> out", so dst = '<name>' falls back to that
if ~block_exists([sys '/' dst]) && block_exists([sys '/' dst ' out'])
    dst = [dst ' out'];
end
hs = get_param([sys '/' src], 'PortHandles');
hd = get_param([sys '/' dst], 'PortHandles');
add_line(sys, hs.Outport(sp), hd.Inport(dp), 'autorouting', 'smart');
end

function tf = block_exists(path)
try
    get_param(path, 'Handle');
    tf = true;
catch
    tf = false;
end
end

function W2(sys, src, dst, dp)
W(sys, src{1}, src{2}, dst, dp);
end
