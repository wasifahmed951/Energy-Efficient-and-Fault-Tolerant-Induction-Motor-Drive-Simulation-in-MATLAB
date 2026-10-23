function P = build_flux_lut(P, source, csvfile)
%BUILD_FLUX_LUT  Fill P.lut.psi with the loss-minimising rotor flux.
%   P = build_flux_lut(P, 'analytic')               from loss_model_ss
%   P = build_flux_lut(P, 'sweep', 'results/flux_opt.csv')   from run_flux_sweep
%   P = build_flux_lut(P, 'csv', 'results/ml_lut.csv')       e.g. exported by Python
%       (columns w, T, psi on the P.lut grid)
if nargin < 2, source = 'analytic'; end
wg = P.lut.w;  Tg = P.lut.T;
Z = zeros(numel(wg), numel(Tg));
switch lower(source)
    case 'analytic'
        for i = 1:numel(wg)
            for j = 1:numel(Tg)
                Z(i,j) = optimal_flux_analytic(wg(i), Tg(j), P);
            end
        end
    case {'sweep', 'csv'}
        D = csvread(csvfile, 1, 0);           % header row skipped
        % columns: w, T, psi_opt, ...  (scattered) -> interpolate on the grid
        F = scatteredInterpolant(D(:,1), D(:,2), D(:,3), 'linear', 'nearest');
        [WW, TT] = ndgrid(wg, Tg);
        Z = F(WW, TT);
    otherwise
        error('build_flux_lut: unknown source %s', source);
end
P.lut.psi = min(max(Z, P.ctrl.psi_min), P.m.psi_r_rated);
end
