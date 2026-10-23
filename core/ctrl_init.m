function st = ctrl_init(P)
%CTRL_INIT  Initial controller state (same struct for IFOC and DTC so it
%   can live in one persistent variable inside a MATLAB Function block).
st.k        = 0;
st.int_w    = 0;          % speed PI integrator [N m]
st.relay    = 1;          % relay output sign (ZN relay test)
st.Te_ref   = 0;
st.Te_ref_f = 0;          % filtered torque reference (transient detection)
st.trans_t  = 0;          % rated-flux hold timer after a transient [s]
st.Te_lim   = P.ctrl.Te_max;
st.psi_ref  = 0;          % ramped rotor-flux reference [Wb]
st.dpsi     = 0;          % its slope [Wb/s]
% IFOC
st.theta_e  = 0;
st.psir_est = 0;
st.int_d    = 0;
st.int_q    = 0;
st.id_ref   = 0;
st.iq_ref   = 0;
st.id       = 0;
st.iq       = 0;
% DTC
st.psis     = zeros(2,1);
st.psr_cm   = zeros(2,1);   % rotor flux, current model (DTC flux observer)
st.i_prev   = zeros(2,1);
st.cpsi     = 1;
st.cT       = 0;
st.legs     = zeros(3,1);
st.Te_est   = 0;
st.psis_ref = 0;
% applied voltage (for estimators)
st.vp_app   = zeros(3,1);
st.v_app    = zeros(2,1);
% current observer [psi_s_a; psi_s_b; psi_r_a; psi_r_b]
st.obs      = zeros(4,1);
st.ia_hat   = 0;
% MRAS speed estimator
st.mr_psv   = zeros(2,1);   % stator flux, voltage model with LPF
st.mr_pri   = zeros(2,1);   % rotor flux, current model
st.mr_lpf   = zeros(2,1);   % LPF state used to high-pass the current model
st.mr_int   = 0;
st.w_est    = 0;            % mechanical [rad/s]
st.mras_t   = 0;            % time spent above 10% speed (MRAS valid region) [s]
% fault detection
st.hw       = zeros(3,1);   % last three speed readings
st.hi       = zeros(3,1);   % last three ia readings
st.cnt_w    = 0;
st.cnt_i    = 0;
st.anom     = 0;            % phase-a anomaly found, classification pending
st.anom_t   = 0;
st.sum_kcl  = 0;
st.sum_exp  = 0;
st.f_speed  = 0;            % latched: speed-sensor fault
st.f_cur    = 0;            % latched: phase-a current-sensor fault
st.f_open   = 0;            % latched: phase a open
st.neutral  = 0;            % neutral contactor command
end
