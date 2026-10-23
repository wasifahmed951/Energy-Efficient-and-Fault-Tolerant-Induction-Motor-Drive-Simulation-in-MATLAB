function t_studies(which)
cd('..'); setup_project(); cd('tests');
Pi = default_params('IFOC'); Pd = default_params('DTC'); wr = Pi.m.wm_rated; Tr = Pi.m.T_rated;
switch which
 case 'zn'
   [P2, info] = run_zn_relay(Pi, 'matlab');
   J = tuning_cost([], P2, 'matlab'); fprintf('ZN IFOC cost J = %.4f\n', J);
 case 'pso'
   o = run_tuning(Pi, 'pso', struct('budget', 6, 'pop', 3, 'seed', 1, 'engine', 'matlab'));
   disp(o.hist)
 case 'eff'
   S = load('P_lut.mat'); Pi.lut = S.P.lut;
   T = run_efficiency_compare(Pi, struct('engine','matlab','w',0.6*wr,'T',0.25*Tr));
   disp(T.cols); disp(T.data)
 case 'sweep'
   T = run_flux_sweep(Pi, struct('engine','matlab','w',0.6*wr,'T',0.25*Tr,'psi',[0.6 0.95], 'out', '../results/test_sweep.csv'));
   disp(T.opt)
 case 'ds'
   DS = export_fault_dataset(Pi, Pd, struct('engine','matlab','speeds',0.5*wr,'loads',0.5*Tr,'onsets',0.5));
   fprintf('X size %s, labels %s\n', mat2str(size(DS.X)), mat2str(DS.y'));
 case 'dtcband'
   [P2, info] = calibrate_dtc_bands(Pd, 5000, 'matlab');
end
end
