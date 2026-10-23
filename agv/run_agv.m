function res = run_agv(P, engine)
%RUN_AGV  AGV warehouse mission with two induction-motor wheel drives (RQ6).
%
%   res = run_agv(P)              Simulink model AGV_MISSION (default)
%   res = run_agv(P, 'matlab')    same functions in a plain MATLAB loop
%
%   Set the drive type with P.ctrl.type (default_params('IFOC'/'DTC')),
%   the flux strategy with P.ctrl.flux_mode, and a fault with
%   P.agv.fault_type / fault_side / fault_time.
%   res.la : AGV log (see agv_step), res.lpL/lpR, res.lcL/lcR : drive logs
%   res.M  : mission metrics (agv_metrics)
if nargin < 2, engine = 'simulink'; end
switch lower(engine)
    case 'matlab'
        Ts = P.ctrl.Ts;  N = round(P.agv.Tend/Ts);  dec = P.agv.log_decim;
        nl = floor(N/dec);
        la = zeros(nl,16); lpL = zeros(nl,23); lpR = lpL; lcL = zeros(nl,16); lcR = lcL; t = zeros(nl,1);
        cL = ctrl_init(P);  cR = ctrl_init(P);  pL = plant_init(P);  pR = plant_init(P);
        ast = agv_init(P);
        mL = zeros(4,1);  mR = zeros(4,1);
        ypL = zeros(23,1); ypR = ypL; ycL = zeros(16,1); ycR = ycL;
        j = 0;  k_done = Inf;
        for k = 1:N
            [refL, refR, loga, ast] = agv_step(ast, ypL, ycL, ypR, ycR, P);
            [uL, ycL, cL] = ctrl_step(cL, mL, refL, P);
            [uR, ycR, cR] = ctrl_step(cR, mR, refR, P);
            [mL, ypL, pL] = plant_step(pL, uL, refL, P);
            [mR, ypR, pR] = plant_step(pR, uR, refR, P);
            if mod(k, dec) == 0
                j = j + 1;
                la(j,:) = loga';  lpL(j,:) = ypL';  lpR(j,:) = ypR';  lcL(j,:) = ycL';  lcR(j,:) = ycR';
                t(j) = k*Ts;
            end
            if ast.done > 0 && isinf(k_done), k_done = k; end
            if k > k_done + round(0.5/Ts), break; end
        end
        res = struct('t', t(1:j), 'la', la(1:j,:), 'lpL', lpL(1:j,:), 'lpR', lpR(1:j,:), ...
                     'lcL', lcL(1:j,:), 'lcR', lcR(1:j,:));
    case 'simulink'
        mdl = 'AGV_MISSION';
        if ~exist([mdl '.slx'], 'file'), build_agv_model(mdl); end
        load_system(mdl);
        in = Simulink.SimulationInput(mdl);
        in = in.setVariable('P', P);
        out = sim(in);
        g = @(n) out.get(n);
        A = g('log_agv');
        res.t = A.time(:);
        res.la = flat(A.signals.values);
        res.lpL = flat(g('log_plantL').signals.values);  res.lpR = flat(g('log_plantR').signals.values);
        res.lcL = flat(g('log_ctrlL').signals.values);   res.lcR = flat(g('log_ctrlR').signals.values);
        n = min([numel(res.t) size(res.la,1) size(res.lpL,1)]);
        res.t = res.t(1:n);  res.la = res.la(1:n,:);
        res.lpL = res.lpL(1:n,:);  res.lpR = res.lpR(1:n,:);  res.lcL = res.lcL(1:n,:);  res.lcR = res.lcR(1:n,:);
    otherwise
        error('run_agv: engine must be ''simulink'' or ''matlab''');
end
res.P = P;
res.M = agv_metrics(res);
end

function M = flat(v)
if ndims(v) == 3
    M = permute(v, [3 1 2]);  M = reshape(M, size(M,1), []);
else
    M = v;
end
end
