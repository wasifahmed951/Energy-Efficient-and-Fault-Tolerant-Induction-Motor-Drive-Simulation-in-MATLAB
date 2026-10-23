%% run_ml_flux_agv.m  -  AGV mission with the ML flux references from Python
% Loads the models chosen in 01_flux_models.ipynb:
%   IFOC -> KAN   (look-up table  python_out/lut_kan_IFOC.csv,  flux_mode 3)
%   DTC  -> MLP   (weights        python_out/mlp_DTC.mat,       flux_mode 4)
% and runs one healthy AGV mission per drive with RATED flux and with the ML flux,
% then prints energy per mission and the saving.
%
% Put this file in the main "Electrical Drive" folder and run:  run_ml_flux_agv
% NOTE: speed-PI gains below are the ZN values. If section 9 of main_run_all
%       uses the GA/PSO gains, put those numbers here instead.

setup_project;
bdclose all; clear functions;
gains = struct('IFOC', [7.46 1993], 'DTC', [7.95 2509]);
outdir = 'python_out';
assert(isfolder(outdir), 'python_out folder not found - run this from the Electrical Drive folder');

rows = {};
for ty = {'IFOC', 'DTC'}
    ty = ty{1};
    for fm = {'rated', 'ML'}
        fm = fm{1};
        P = default_params(ty);
        P.ctrl.Kp_w = gains.(ty)(1);  P.ctrl.Ki_w = gains.(ty)(2);
        P.agv.fault_type = 0;
        if strcmp(fm, 'rated')
            P.ctrl.flux_mode = 1;  label = 'rated flux';
        elseif strcmp(ty, 'IFOC')
            P = build_flux_lut(P, 'csv', fullfile(outdir, 'lut_kan_IFOC.csv'));   % KAN as look-up table
            P.ctrl.flux_mode = 3;  label = 'KAN (LUT)';
        else
            S = load(fullfile(outdir, 'mlp_DTC.mat'));  P.mlp = S.mlp;            % MLP weights
            P.ctrl.flux_mode = 4;  label = 'MLP 2-10-1';
        end
        fprintf('\nRunning %s with %s ...\n', ty, label);
        res = run_agv(P, 'simulink');
        M = res.M;
        rows(end+1, :) = {ty, label, M.completed, M.mission_time, M.E_Wh, M.e_cross_max_cm}; %#ok<SAGROW>
        fprintf('  done %d, %.1f s, %.2f Wh, e_max %.1f cm\n', M.completed, M.mission_time, M.E_Wh, M.e_cross_max_cm);
    end
end

T = cell2table(rows, 'VariableNames', {'drive', 'flux', 'completed', 'time_s', 'energy_Wh', 'e_max_cm'});
fprintf('\n================ AGV energy per mission ================\n');
disp(T);
for ty = {'IFOC', 'DTC'}
    e = T.energy_Wh(strcmp(T.drive, ty{1}));
    fprintf('%s: ML flux saves %.1f %% energy per mission vs rated flux\n', ty{1}, 100*(e(1) - e(2))/e(1));
end
writetable(T, fullfile('results', 'agv_ml_flux.csv'));
fprintf('\nSaved results/agv_ml_flux.csv.\n');
