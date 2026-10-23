function P = default_params(ctrl_type)
%DEFAULT_PARAMS  All parameters for the induction-motor drive thesis.
%   P = default_params('IFOC')  or  P = default_params('DTC')
%
%   Motor: 4 kW, 400 V (line, rms), 50 Hz, 4-pole squirrel-cage machine.
%   The electrical parameters are the Simscape "4 kW, 400 V, 50 Hz,
%   1430 rpm" preset values. The saturation curve and core-loss
%   coefficients are PLACEHOLDERS: replace them with values fitted to
%   your machine's no-load test (see README, section "Parameters").
%
%   Every field is numeric (no strings) so the struct can be passed to
%   MATLAB Function blocks as a Simulink parameter.

if nargin < 1, ctrl_type = 'IFOC'; end

%% ---------------- Motor (per phase, referred to stator) ----------------
m.Rs   = 1.405;      % stator resistance [Ohm]
m.Rr   = 1.395;      % rotor resistance [Ohm]
m.Lls  = 0.005839;   % stator leakage [H]
m.Llr  = 0.005839;   % rotor leakage [H]
m.Lm_rated = 0.1722; % magnetizing inductance at rated flux (secant) [H]
m.p    = 2;          % pole pairs
m.J    = 0.0131;     % rotor inertia [kg m^2]
m.B    = 0.002985;   % viscous friction [N m s]
m.Prated = 4000;     % [W]
m.nrated = 1430;     % [rpm]
m.Vll    = 400;      % [V rms, line]
m.fn     = 50;       % [Hz]

% Saturation: secant inductance  Lms(|psi_m|) = Lm0 / (1 + a*(|psi_m|/psi_k)^b)
% Lm0 is chosen so that Lms(psi_k) = Lm_rated.
m.sat_on = 1;        % 1 = saturation modelled, 0 = linear (Lm = Lm_rated)
m.sat_a  = 0.15;     % PLACEHOLDER: fit to no-load test
m.sat_b  = 6;        % PLACEHOLDER: fit to no-load test
m.psi_k  = 0.95;     % knee = rated air-gap flux (peak, amplitude-invariant) [Wb]
m.Lm0    = m.Lm_rated * (1 + m.sat_a);

% Derived
m.Ls = m.Lls + m.Lm_rated;
m.Lr = m.Llr + m.Lm_rated;
m.sigma = 1 - m.Lm_rated^2/(m.Ls*m.Lr);
m.tau_r = m.Lr/m.Rr;
m.wm_rated = m.nrated*2*pi/60;          % [rad/s mech]
m.T_rated  = m.Prated/m.wm_rated;       % [N m]  (about 26.7)
m.psi_r_rated = 0.95;                   % rated rotor flux (peak) [Wb]
m.I_max = 20;                           % current limit (peak per phase) [A]

% Core loss  P_fe = (kh*fe + ke*fe^2) * |psi_m|^2   (PLACEHOLDERS, ~120 W at rated)
m.kh = 1.86;     % [W/(Hz Wb^2)]
m.ke = 0.016;    % [W/(Hz^2 Wb^2)]
P.m = m;

%% ---------------- Inverter ----------------
inv.Vdc  = 600;      % DC link [V]
inv.fsw  = 5000;     % IFOC carrier frequency [Hz]
inv.Vce0 = 1.0;      % IGBT/diode threshold voltage [V]
inv.rce  = 0.04;     % IGBT/diode slope resistance [Ohm]
inv.Esw  = 1.2e-3;   % switching energy per transition pair at Vref, Iref [J]
inv.Vref = 600;
inv.Iref = 10;
P.inv = inv;

%% ---------------- Simulation ----------------
s.h_max     = 50e-6;   % max RK4 step inside a PWM segment [s]
s.sat_iter  = 3;       % fixed-point iterations for saturation
s.log_decim = 1;       % To Workspace decimation (1 = every step)
P.sim = s;

%% ---------------- Controller (common) ----------------
c.type = 1;                         % 1 = IFOC, 2 = DTC
if strcmpi(ctrl_type,'DTC'), c.type = 2; end
c.Ts_ifoc = 1/(2*inv.fsw);          % double-update PWM: 100 us
c.Ts_dtc  = 25e-6;                  % DTC sampling: 40 kHz
c.Ts = c.Ts_ifoc; if c.type == 2, c.Ts = c.Ts_dtc; end

% Speed loop (both drives). Placeholder gains; tuning replaces them.
c.Kp_w = 1.0;        % [N m / (rad/s)]
c.Ki_w = 20;         % [N m / rad]
c.Te_max = 1.5*m.T_rated;
c.speed_mode = 1;    % 1 = PI, 2 = relay (Ziegler-Nichols relay test)
c.relay_T = 0.3*m.T_rated;   % relay amplitude [N m]
c.relay_h = 0.5;             % relay hysteresis [rad/s]
c.relay_bias = 0;            % torque bias during relay test [N m]

% IFOC current loops: pole-zero cancellation at bandwidth wbw
wbw = 2*pi*500;
c.Kp_i = wbw*m.sigma*m.Ls;
c.Ki_i = wbw*(m.Rs + m.Rr*(m.Lm_rated/m.Lr)^2);
c.ctrl_sat_aware = 1;        % controller uses Lms(psi) instead of Lm_rated

% DTC hysteresis bands (calibrate with calibrate_dtc_bands.m)
c.h_psi = 0.0070;    % flux band [Wb]  (calibrated to ~5 kHz average switching)
c.h_T   = 1.05;      % torque band [N m] (calibrated to ~5 kHz average switching)
c.wc_int = 1.0;      % (no longer used by DTC; kept for reference) [rad/s]
c.wc_cm  = 2*pi*2;   % DTC flux observer crossover: current model below ~2 Hz,
                     % voltage model above [rad/s]

% Flux reference
c.flux_mode = 1;     % 1 = rated, 2 = fixed value, 3 = look-up table, 4 = MLP
c.psi_fixed = m.psi_r_rated;
c.psi_min   = 0.35*m.psi_r_rated;
c.psi_rate  = 5.0;   % max flux-reference slope [Wb/s]
c.trans_dw  = 0.05*m.wm_rated;   % speed error that counts as a transient [rad/s]
c.trans_dT  = 0.25*m.T_rated;    % torque-ref jump that counts as a transient [N m]
c.trans_hold = 0.15; % hold rated flux this long after a transient [s]

% Observers
c.Ko = 10;           % current-observer correction gain [Ohm]
c.mras_Kp = 400;     % MRAS PI (normalised)
c.mras_Ki = 4e4;
c.mras_wc = 2*pi*1;  % LPF in the MRAS voltage model [rad/s]

% Fault detection
c.fd_on       = 1;                 % 1 = detection and reconfiguration active
c.tdo_w_thr   = 0.2*m.wm_rated;    % third-difference threshold, speed [rad/s]
c.tdo_i_thr   = 6;                 % third-difference threshold, current [A]
c.res_w_thr   = 0.15*m.wm_rated;   % speed residual threshold [rad/s]
c.res_i_thr   = 2.5;               % current residual threshold [A]
c.res_N       = 40;                % consecutive samples above threshold
c.open_thr    = 0.5;               % |ia| below this [A] ...
c.open_ref    = 3.0;               % ... while |ia_ref| above this [A]
c.open_N_time = 4e-3;              % ... for this long [s]
c.remedy_open = 1;                 % IFOC: 1 = connect neutral + remap voltages
c.derate_open = 0.5;               % torque-limit factor after an open phase
c.derate_sensor = 1.0;             % torque-limit factor after a sensor fault
P.ctrl = c;

%% ---------------- Flux look-up table (filled by build_flux_lut) ----------------
lut.w  = linspace(0.1, 1.0, 10) * m.wm_rated;     % [rad/s]
lut.T  = linspace(0.05, 1.0, 10) * m.T_rated;     % [N m]
lut.psi = m.psi_r_rated * ones(10,10);            % rotor flux [Wb] (rows = w, cols = T)
P.lut = lut;

%% ---------------- MLP weights (filled later from Python) ----------------
% y = W2 * tanh(W1 * x_n + b1) + b2,  x_n = (x - x_mu)./x_sd,  psi = y*y_sd + y_mu
nh = 10;
mlp.W1 = zeros(nh,2); mlp.b1 = zeros(nh,1);
mlp.W2 = zeros(1,nh); mlp.b2 = 0;
mlp.x_mu = [0.5*m.wm_rated; 0.5*m.T_rated]; mlp.x_sd = [0.3*m.wm_rated; 0.3*m.T_rated];
mlp.y_mu = m.psi_r_rated; mlp.y_sd = 0.1;
P.mlp = mlp;

%% ---------------- AGV (robotics case study) ----------------
a.m_robot   = 800;     % robot mass [kg]        PLACEHOLDER: take from a datasheet
a.m_payload = 1500;    % towed payload [kg]     PLACEHOLDER
a.r_wheel   = 0.15;    % wheel radius [m]
a.W_track   = 0.70;    % track width [m]
a.G         = 20;      % gear ratio motor:wheel
a.eta_g     = 0.95;    % gear efficiency
a.Crr       = 0.015;   % rolling-resistance coefficient
a.v_max     = 1.0;     % [m/s]
a.a_max     = 0.4;     % [m/s^2]
a.a_lat     = 0.3;     % lateral acceleration limit in corners [m/s^2]
a.L_look    = 0.8;     % pure-pursuit look-ahead [m]
a.dwell     = 3.0;     % stop time at each station [s]
a.E_batt_Wh = 10000;   % usable battery energy [Wh]
a.v_limp    = 0.4;     % speed limit in limp-home mode [m/s]
a.Lx        = 12;      % route length [m]
a.Ly        = 6;       % route width [m]
a.t_start   = 0.4;     % wait for magnetisation before moving [s]
a.Tend      = 80;      % maximum mission time [s]
a.fault_type = 0;      % 0 none, 1 speed sensor, 2 phase-a current sensor, 3 open phase a
a.fault_side = 2;      % 1 left motor, 2 right motor
a.fault_time = 20;     % [s]
a.log_decim  = 20;     % log every 20th sample in AGV runs
P.agv = a;
end
