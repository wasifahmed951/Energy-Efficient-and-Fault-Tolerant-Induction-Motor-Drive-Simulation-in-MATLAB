function pst = plant_init(P)
%PLANT_INIT  Initial state of the inverter + induction-motor plant (motor at rest, no flux).
pst.x     = zeros(6,1);   % [lam_alpha; psi_s_beta; psi_r_alpha; psi_r_beta; w_m; theta_m]
pst.psim  = zeros(2,1);   % air-gap flux (warm start for saturation solve)
pst.mode  = 1;            % 1 healthy, 2 phase a open (isolated neutral), 3 phase a open + neutral connected
pst.legs  = zeros(3,1);   % leg states at the end of the last step
pst.half  = 0;            % PWM half-period index (0 = rising carrier, 1 = falling)
pst.ang_m = 0;            % angle of psi_m at the end of the last step
pst.x(5)  = P.m.wm_rated*0; % start at standstill
end
