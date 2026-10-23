function t_agv(fault)
addpath('../params','../core','../sim','../analysis','../efficiency','../utils','../agv');
P = default_params('IFOC'); S = load('P_lut.mat'); P.lut = S.P.lut;   % IFOC with analytic flux LUT
P.ctrl.flux_mode = 3;
P.agv.Lx = 1.5; P.agv.Ly = 0.75; P.agv.v_max = 0.5; P.agv.dwell = 1; P.agv.L_look = 0.5; P.agv.Tend = 18;
P.agv.fault_type = fault; P.agv.fault_side = 2; P.agv.fault_time = 4;
tic; res = run_agv(P, 'matlab'); toc
M = res.M; disp(M)
save('-mat', sprintf('res_agv_%d.mat', fault), 'res');
end
